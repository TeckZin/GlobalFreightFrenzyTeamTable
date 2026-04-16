"""Multi-modal transportation routing with clustering, scoring, and ocean waypoints.

Strategy:
* **Tick 0** — Cluster boxes by (origin hub, destination hub). Score each cluster by
  boxes_delivered / total_cost. For each cluster, pick the cheapest vehicle type given
  terrain constraints and capacity. For ocean legs, route through canal/strait
  waypoints using Dijkstra to avoid cutting through continents.
* **Every tick** — Drive each vehicle through its assigned route (multi-leg for ocean
  ships). Unload at destination, load new cargo opportunistically.
"""
from simulator import VehicleType, haversine_distance_meters
import heapq

_PROXIMITY_M = 50.0
_FACILITY_RADIUS_M = 5000.0
DEBUG = True


# ── Ocean routing waypoints ──────────────────────────────────────────────
# Chokepoints + open-ocean pivots so ships don't cut through land.
OCEAN_WAYPOINTS = {
    "panama":       (9.08, -79.68),
    "suez":         (30.0, 32.55),
    "gibraltar":    (35.95, -5.6),
    "malacca":      (2.5, 101.5),
    "good_hope":    (-34.4, 18.5),
    "horn":         (-55.98, -67.27),
    "mid_pacific":  (0.0, -150.0),
    "mid_atlantic": (0.0, -30.0),
    "mid_indian":   (0.0, 75.0),
    "n_atlantic":   (40.0, -40.0),
    "n_pacific":    (40.0, -170.0),
}

# Edges = pairs of waypoints with clear water between them.
OCEAN_EDGES = [
    ("panama", "mid_pacific"),
    ("panama", "mid_atlantic"),
    ("panama", "n_pacific"),
    ("suez", "gibraltar"),
    ("suez", "mid_indian"),
    ("gibraltar", "mid_atlantic"),
    ("gibraltar", "n_atlantic"),
    ("malacca", "mid_indian"),
    ("malacca", "mid_pacific"),
    ("good_hope", "mid_atlantic"),
    ("good_hope", "mid_indian"),
    ("horn", "mid_pacific"),
    ("horn", "mid_atlantic"),
    ("mid_atlantic", "n_atlantic"),
    ("mid_pacific", "n_pacific"),
]


# ── Vehicle lookup helpers ───────────────────────────────────────────────
VEHICLE_MODE = {
    "SemiTruck": "land",
    "Train":     "land",
    "Airplane":  "air",
    "CargoShip": "ocean",
    "Drone":     "air",
}


def log(*args):
    if DEBUG:
        print(*args)


def km(a, b):
    """Great-circle distance in kilometres."""
    return haversine_distance_meters(a, b) / 1000.0


# ── Persistent state across ticks (module-level) ─────────────────────────
# Each vehicle gets an ordered list of waypoint coordinates to visit.
_vehicle_routes = {}   # vid -> [coord, coord, ...]  (next destination is [0])
_cluster_plan   = None # computed once at tick 0


# ── Planning: cluster boxes and assign vehicles ──────────────────────────
def _cluster_boxes(boxes):
    """Group boxes by (origin, destination). Returns dict keyed by pair."""
    clusters = {}
    for box in boxes.values():
        if box["delivered"] or box["vehicle_id"] is not None:
            continue
        key = (box["location"], box["destination"])
        clusters.setdefault(key, []).append(box["id"])
    return clusters


def _leg_feasible(mode, origin, dest, airports, ocean_ports):
    """Can a vehicle of this mode legally fly/drive/sail this leg?"""
    if mode == "land":
        return True  # land vehicles can move anywhere; terrain penalty handles water
    if mode == "air":
        # Need airport at both ends for load/unload
        near_origin = any(km(origin, a) * 1000 <= _FACILITY_RADIUS_M for a in airports)
        near_dest   = any(km(dest,   a) * 1000 <= _FACILITY_RADIUS_M for a in airports)
        return near_origin and near_dest
    if mode == "ocean":
        if not ocean_ports:
            return False
        near_origin = any(km(origin, p) * 1000 <= _FACILITY_RADIUS_M for p in ocean_ports)
        near_dest   = any(km(dest,   p) * 1000 <= _FACILITY_RADIUS_M for p in ocean_ports)
        return near_origin and near_dest
    return False


def _ocean_route(origin_port, dest_port):
    """Dijkstra through OCEAN_WAYPOINTS. Returns [origin, waypoint, ..., dest]."""
    # Build a combined graph: ports + waypoints, fully connected for ports↔waypoints,
    # waypoint↔waypoint only along OCEAN_EDGES.
    nodes = {"__origin__": origin_port, "__dest__": dest_port}
    nodes.update(OCEAN_WAYPOINTS)

    adj = {n: [] for n in nodes}
    # Ports connect to all waypoints (they can reach open water from any port).
    for wp in OCEAN_WAYPOINTS:
        adj["__origin__"].append(wp)
        adj[wp].append("__origin__")
        adj["__dest__"].append(wp)
        adj[wp].append("__dest__")
    # Direct port-to-port for short hops.
    adj["__origin__"].append("__dest__")
    adj["__dest__"].append("__origin__")
    # Waypoint edges.
    for a, b in OCEAN_EDGES:
        adj[a].append(b)
        adj[b].append(a)

    # Dijkstra
    dist = {n: float("inf") for n in nodes}
    prev = {n: None for n in nodes}
    dist["__origin__"] = 0.0
    pq = [(0.0, "__origin__")]
    while pq:
        d, u = heapq.heappop(pq)
        if u == "__dest__":
            break
        if d > dist[u]:
            continue
        for v in adj[u]:
            w = km(nodes[u], nodes[v])
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    # Reconstruct path
    path = []
    cur = "__dest__"
    while cur is not None:
        path.append(nodes[cur])
        cur = prev[cur]
    path.reverse()
    return path


def _nearest(loc, points):
    """Return the point from iterable closest to loc."""
    return min(points, key=lambda p: km(loc, p))


def _score_cluster(n_boxes, total_cost):
    """Higher is better. Undelivered penalty is 1000/box, so delivering is huge."""
    if total_cost <= 0:
        return float("inf")
    # Value of delivering = avoided penalty; cost = what we pay.
    return (n_boxes * 1000.0) / total_cost


def _plan_cluster(origin, dest, n_boxes, airports, ocean_ports):
    """Pick best vehicle type + compute route (list of waypoints) + cost estimate."""
    direct_km = km(origin, dest)
    candidates = []

    for vt in VehicleType:
        name = vt.name
        cfg = vt.value
        mode = VEHICLE_MODE[name]

        if not _leg_feasible(mode, origin, dest, airports, ocean_ports):
            continue

        # Number of vehicles needed (integer division, round up).
        n_vehicles = (n_boxes + cfg.capacity - 1) // cfg.capacity

        # Compute route for this mode.
        if mode == "ocean":
            origin_port = _nearest(origin, ocean_ports)
            dest_port   = _nearest(dest,   ocean_ports)
            # Ship route: origin_port → waypoints → dest_port
            ship_path = _ocean_route(origin_port, dest_port)
            # Distance the ship itself travels
            ship_km = sum(km(ship_path[i], ship_path[i + 1])
                          for i in range(len(ship_path) - 1))
            # Still need truck legs: origin → origin_port and dest_port → dest
            # (We'll model this as a multi-leg plan; cost estimate includes trucks.)
            truck_cfg = VehicleType.SemiTruck.value
            truck_km  = km(origin, origin_port) + km(dest_port, dest)
            ship_cost = (cfg.base_cost * n_vehicles
                         + cfg.per_km_cost * ship_km * n_vehicles
                         + n_boxes * 2)  # load + unload at port
            truck_cost = (truck_cfg.base_cost * 2
                          + truck_cfg.per_km_cost * truck_km
                          + n_boxes * 2)
            total_cost = ship_cost + truck_cost + n_boxes  # initial load
            route = {
                "mode": "ocean_multimodal",
                "ship_path": ship_path,
                "origin_port": origin_port,
                "dest_port": dest_port,
                "n_vehicles": n_vehicles,
            }
        elif mode == "air":
            origin_ap = _nearest(origin, airports)
            dest_ap   = _nearest(dest,   airports)
            air_km = km(origin_ap, dest_ap)
            truck_km = km(origin, origin_ap) + km(dest_ap, dest)
            truck_cfg = VehicleType.SemiTruck.value
            air_cost = (cfg.base_cost * n_vehicles
                        + cfg.per_km_cost * air_km * n_vehicles
                        + n_boxes * 2)
            truck_cost = (truck_cfg.base_cost * 2
                          + truck_cfg.per_km_cost * truck_km
                          + n_boxes * 2)
            total_cost = air_cost + truck_cost + n_boxes
            route = {
                "mode": "air_multimodal",
                "origin_ap": origin_ap,
                "dest_ap": dest_ap,
                "n_vehicles": n_vehicles,
            }
        else:  # land — direct
            total_cost = (cfg.base_cost * n_vehicles
                          + cfg.per_km_cost * direct_km * n_vehicles
                          + n_boxes)  # load only, unload free-ish
            route = {
                "mode": "land_direct",
                "n_vehicles": n_vehicles,
            }

        candidates.append((total_cost, name, route))

    if not candidates:
        return None

    candidates.sort(key=lambda c: c[0])
    cost, vehicle_name, route = candidates[0]
    return {
        "vehicle_type": vehicle_name,
        "route": route,
        "est_cost": cost,
        "score": _score_cluster(n_boxes, cost),
        "origin": origin,
        "dest": dest,
        "n_boxes": n_boxes,
    }


def _build_plan(sim_state):
    """Cluster all boxes and produce a prioritized list of delivery plans."""
    boxes = sim_state.get_boxes()
    airports = list(sim_state.get_airports())
    ocean_ports = list(sim_state.get_ocean_ports())

    clusters = _cluster_boxes(boxes)
    plans = []
    for (origin, dest), box_ids in clusters.items():
        plan = _plan_cluster(origin, dest, len(box_ids), airports, ocean_ports)
        if plan is None:
            log(f"  skip cluster {origin}->{dest}: no feasible vehicle")
            continue
        plan["box_ids"] = box_ids
        plans.append(plan)

    # Highest score first (most boxes per dollar).
    plans.sort(key=lambda p: p["score"], reverse=True)
    return plans


# ── Execution: spawn and route vehicles per plan ─────────────────────────
def _spawn_for_plan(sim_state, plan):
    """Create vehicle(s) for a plan and register their route. Returns list of vids."""
    vt_name = plan["vehicle_type"]
    vt = VehicleType[vt_name]
    route = plan["route"]
    origin = plan["origin"]
    vids = []

    if route["mode"] == "land_direct":
        try:
            vid = sim_state.create_vehicle(vt, origin)
        except ValueError as e:
            log(f"  spawn {vt_name} at {origin} failed: {e}")
            return []
        _vehicle_routes[vid] = [plan["dest"]]
        vids.append(vid)

    elif route["mode"] == "air_multimodal":
        # Need truck to hop origin → origin_ap, then plane origin_ap → dest_ap,
        # then truck dest_ap → dest. We'll spawn the plane at origin_ap directly
        # and use trucks for the short hops.
        origin_ap = route["origin_ap"]
        dest_ap   = route["dest_ap"]
        # Truck at origin to carry to airport
        try:
            truck1 = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
            _vehicle_routes[truck1] = [origin_ap]
            vids.append(truck1)
        except ValueError:
            pass
        try:
            plane = sim_state.create_vehicle(vt, origin_ap)
            _vehicle_routes[plane] = [dest_ap]
            vids.append(plane)
        except ValueError as e:
            log(f"  spawn {vt_name} at {origin_ap} failed: {e}")

    elif route["mode"] == "ocean_multimodal":
        origin_port = route["origin_port"]
        dest_port   = route["dest_port"]
        ship_path   = route["ship_path"]
        try:
            truck1 = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
            _vehicle_routes[truck1] = [origin_port]
            vids.append(truck1)
        except ValueError:
            pass
        try:
            ship = sim_state.create_vehicle(vt, origin_port)
            # Route is all waypoints after origin_port
            _vehicle_routes[ship] = list(ship_path[1:])
            vids.append(ship)
        except ValueError as e:
            log(f"  spawn {vt_name} at {origin_port} failed: {e}")

    return vids


def step(sim_state):
    global _cluster_plan
    tick = sim_state.tick
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    # ── Tick 0: build the plan, spawn initial vehicles ──────────────────
    if tick == 0:
        log("=== PLANNING ===")
        _cluster_plan = _build_plan(sim_state)
        log(f"  {len(_cluster_plan)} clusters to route")
        for p in _cluster_plan[:5]:
            log(f"   score={p['score']:.2f}  {p['n_boxes']} boxes  "
                f"{p['vehicle_type']}  est_cost={p['est_cost']:.1f}")

        # Spawn vehicles for every plan. (Greedy: we try all; sim will reject
        # any that violate spawn rules.)
        for plan in _cluster_plan:
            vids = _spawn_for_plan(sim_state, plan)
            # Load boxes onto the FIRST vehicle spawned (the one at origin).
            if vids:
                try:
                    cap = VehicleType[vehicles_type(sim_state, vids[0])].value.capacity
                    sim_state.load_vehicle(vids[0], plan["box_ids"][:cap])
                except (ValueError, KeyError) as e:
                    log(f"  initial load failed for {vids[0]}: {e}")
        return

    # ── Every tick: drive vehicles through their routes ─────────────────
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    for vid, vehicle in vehicles.items():
        loc = vehicle["location"]

        # Still moving? Let the physics engine work.
        if vehicle["destination"] is not None:
            continue

        # 1. Unload any box whose destination is here.
        deliverable = [
            bid for bid in vehicle["cargo"]
            if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M
        ]
        if deliverable:
            try:
                sim_state.unload_vehicle(vid, deliverable)
                boxes = sim_state.get_boxes()
            except ValueError as e:
                log(f"  unload {vid} failed: {e}")

        # 2. Transfer cargo between vehicles at the same location (truck→plane/ship
        #    hand-off). Load anything here that isn't on a vehicle yet.
        cfg = VehicleType[vehicle["vehicle_type"]].value
        remaining = cfg.capacity - len(vehicle["cargo"])
        if remaining > 0:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"]
                and box["vehicle_id"] is None
                and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                try:
                    sim_state.load_vehicle(vid, loadable[:remaining])
                    boxes = sim_state.get_boxes()
                except ValueError as e:
                    log(f"  load {vid} failed: {e}")

        # 3. Advance along the registered route.
        route = _vehicle_routes.get(vid, [])
        # Drop waypoints we've already reached.
        while route and haversine_distance_meters(loc, route[0]) <= _PROXIMITY_M:
            route.pop(0)

        if route:
            try:
                sim_state.move_vehicle(vid, route[0])
            except (ValueError, KeyError) as e:
                log(f"  move {vid} failed: {e}")
        elif vehicle["cargo"]:
            # No route left but still carrying cargo — head to first box's dest.
            next_dest = boxes[vehicle["cargo"][0]]["destination"]
            try:
                sim_state.move_vehicle(vid, next_dest)
            except (ValueError, KeyError) as e:
                log(f"  fallback move {vid} failed: {e}")


def vehicles_type(sim_state, vid):
    """Helper: get vehicle_type string for a given vid."""
    return sim_state.get_vehicles()[vid]["vehicle_type"]
