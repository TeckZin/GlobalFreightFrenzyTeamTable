from simulator import VehicleType, haversine_distance_meters

_PROXIMITY_M = 50.0
_HUB_CLUSTER_RADIUS_M = 5000.0
_DEST_CLUSTER_RADIUS_M = 80000.0
_SHIP_MIN_BOXES = 80
_TRAIN_MIN_BOXES = 40
_AIR_MIN_DISTANCE_M = 1200000.0
_AIR_MIN_BOXES = 35


def _distance_m(a, b):
    return haversine_distance_meters(a, b)


def _distance_km(a, b):
    return _distance_m(a, b) / 1000.0


def _avg_point(points):
    if not points:
        return (0.0, 0.0)
    return (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )


def _cfg(vehicle_type_name):
    return VehicleType[vehicle_type_name].value


def _capacity(vehicle_type_name):
    return getattr(_cfg(vehicle_type_name), "capacity", 0)


def _base_cost(vehicle_type):
    return getattr(vehicle_type.value, "base_cost", 0.0)


def _per_km_cost(vehicle_type):
    return getattr(vehicle_type.value, "per_km_cost", 0.0)


def _speed(vehicle_type):
    cfg = vehicle_type.value
    return getattr(cfg, "speed_km_h", getattr(cfg, "speed", 0.0))


def _get_active_events(sim_state):
    if hasattr(sim_state, "get_active_events"):
        try:
            return sim_state.get_active_events() or []
        except Exception:
            return []
    return []


def _events_block(vehicle_type_name, events):
    for event in events:
        etype = event.get("type")
        if etype == "ground_stop_flights" and vehicle_type_name in ("Airplane", "Drone"):
            return True
        if etype == "oceanic_weather" and vehicle_type_name == "CargoShip":
            return True
    return False


def _boxes_waiting(boxes):
    out = {}
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        out[bid] = box
    return out


def _cluster_hubs(boxes):
    hubs = []

    for bid, box in boxes.items():
        loc = box["location"]
        placed = False

        for hub in hubs:
            if _distance_m(loc, hub["center"]) <= _HUB_CLUSTER_RADIUS_M:
                hub["box_ids"].append(bid)
                hub["points"].append(loc)
                hub["center"] = _avg_point(hub["points"])
                placed = True
                break

        if not placed:
            hubs.append({
                "center": loc,
                "points": [loc],
                "box_ids": [bid],
            })

    return hubs


def _boxes_at_hub(boxes, hub_center):
    out = []
    for bid, box in boxes.items():
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        if _distance_m(box["location"], hub_center) <= _PROXIMITY_M:
            out.append(bid)
    return out


def _deliverable_here(vehicle, boxes):
    loc = vehicle["location"]
    out = []
    for bid in vehicle["cargo"]:
        if bid not in boxes:
            continue
        if boxes[bid]["delivered"]:
            continue
        if _distance_m(loc, boxes[bid]["destination"]) <= _PROXIMITY_M:
            out.append(bid)
    return out


def _group_by_exact_destination(box_ids, boxes):
    groups = {}
    for bid in box_ids:
        dest = boxes[bid]["destination"]
        key = (round(dest[0], 5), round(dest[1], 5))
        if key not in groups:
            groups[key] = {
                "destination": dest,
                "box_ids": [],
            }
        groups[key]["box_ids"].append(bid)
    return sorted(groups.values(), key=lambda g: len(g["box_ids"]), reverse=True)


def _group_by_destination_region(box_ids, boxes):
    clusters = []

    for bid in box_ids:
        dest = boxes[bid]["destination"]
        placed = False

        for cluster in clusters:
            if _distance_m(dest, cluster["center"]) <= _DEST_CLUSTER_RADIUS_M:
                cluster["box_ids"].append(bid)
                pts = [boxes[x]["destination"] for x in cluster["box_ids"]]
                cluster["center"] = _avg_point(pts)
                placed = True
                break

        if not placed:
            clusters.append({
                "center": dest,
                "box_ids": [bid],
            })

    clusters.sort(key=lambda c: len(c["box_ids"]), reverse=True)
    return clusters


def _find_nearest_hub(point, hub_centers, exclude=None):
    best = None
    best_dist = float("inf")
    for hub in hub_centers:
        if exclude is not None and _distance_m(hub, exclude) <= _PROXIMITY_M:
            continue
        d = _distance_m(point, hub)
        if d < best_dist:
            best_dist = d
            best = hub
    return best, best_dist


def _safe_create_vehicle(sim_state, vehicle_type, location):
    try:
        vid = sim_state.create_vehicle(vehicle_type, location)
        print(f"created {vid} type={vehicle_type.name} at {location}")
        return vid
    except Exception as e:
        print(f"failed create {vehicle_type.name} at {location}: {e}")
        return None


def _choose_vehicle_for_leg(sim_state, origin, target, box_count, prefer_ship=False):
    events = _get_active_events(sim_state)
    dist_m = _distance_m(origin, target)
    dist_km = dist_m / 1000.0

    candidates = []

    if prefer_ship and not _events_block("CargoShip", events) and box_count >= _SHIP_MIN_BOXES:
        candidates.append(VehicleType.CargoShip)

    if box_count >= _TRAIN_MIN_BOXES:
        candidates.append(VehicleType.Train)

    candidates.append(VehicleType.SemiTruck)

    if dist_m >= _AIR_MIN_DISTANCE_M and box_count >= _AIR_MIN_BOXES:
        if not _events_block("Airplane", events):
            candidates.append(VehicleType.Airplane)

    if box_count <= 5 and dist_m <= 30000 and not _events_block("Drone", events):
        candidates.append(VehicleType.Drone)

    scored = []
    for vtype in candidates:
        cost = _base_cost(vtype) + _per_km_cost(vtype) * dist_km
        speed = max(_speed(vtype), 1.0)
        score = (box_count * speed) / max(cost, 1.0)
        scored.append((score, vtype))

    scored.sort(key=lambda x: x[0], reverse=True)

    for _, vtype in scored:
        vid = _safe_create_vehicle(sim_state, vtype, origin)
        if vid is not None:
            return vid

    return None


def _load_up_to_capacity(sim_state, vid, vehicle, candidate_box_ids):
    remaining = max(0, _capacity(vehicle["vehicle_type"]) - len(vehicle["cargo"]))
    if remaining <= 0:
        return 0

    chosen = candidate_box_ids[:remaining]
    if not chosen:
        return 0

    try:
        sim_state.load_vehicle(vid, chosen)
        print(f"loaded {len(chosen)} onto {vid}")
        return len(chosen)
    except Exception as e:
        print(f"failed load {vid}: {e}")
        return 0


def _choose_ship_transfer_target(hub_center, region_center, all_hub_centers):
    target_hub, _ = _find_nearest_hub(region_center, all_hub_centers, exclude=hub_center)
    return target_hub


def _spawn_initial(sim_state):
    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    hub_centers = [h["center"] for h in hubs]

    print(f"tick0 hubs={len(hubs)}")

    for hub in hubs:
        hub_center = hub["center"]
        waiting_ids = _boxes_at_hub(sim_state.get_boxes(), hub_center)
        if not waiting_ids:
            continue

        region_groups = _group_by_destination_region(waiting_ids, sim_state.get_boxes())
        if not region_groups:
            continue

        best_region = region_groups[0]
        region_center = best_region["center"]
        region_box_count = len(best_region["box_ids"])

        nearest_dest_hub, nearest_dist = _find_nearest_hub(region_center, hub_centers, exclude=hub_center)

        prefer_ship = False
        planned_target = region_center

        if nearest_dest_hub is not None:
            origin_to_region = _distance_m(hub_center, region_center)
            origin_to_hub = _distance_m(hub_center, nearest_dest_hub)

            if origin_to_region > 300000 and origin_to_hub > 100000 and region_box_count >= _SHIP_MIN_BOXES:
                prefer_ship = True
                planned_target = nearest_dest_hub

        vid = _choose_vehicle_for_leg(
            sim_state,
            hub_center,
            planned_target,
            region_box_count,
            prefer_ship=prefer_ship,
        )
        if vid is None:
            continue

        vehicles = sim_state.get_vehicles()
        if vid not in vehicles:
            continue
        vehicle = vehicles[vid]

        if prefer_ship:
            candidate_ids = best_region["box_ids"]
        else:
            exact_groups = _group_by_exact_destination(waiting_ids, sim_state.get_boxes())
            candidate_ids = exact_groups[0]["box_ids"] if exact_groups else waiting_ids

        _load_up_to_capacity(sim_state, vid, vehicle, candidate_ids)

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if not vehicle or not vehicle["cargo"]:
            continue

        try:
            sim_state.move_vehicle(vid, planned_target)
            print(f"moving {vid} to {planned_target}")
        except Exception as e:
            print(f"failed move {vid}: {e}")


def _manage_idle_vehicle(sim_state, vid, vehicle, all_hub_centers):
    boxes = sim_state.get_boxes()
    loc = vehicle["location"]

    deliverable = _deliverable_here(vehicle, boxes)
    if deliverable:
        try:
            sim_state.unload_vehicle(vid, deliverable)
            print(f"unloaded {len(deliverable)} from {vid}")
        except Exception as e:
            print(f"failed unload {vid}: {e}")

        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            return

    if vehicle["cargo"]:
        groups = _group_by_exact_destination(vehicle["cargo"], boxes)
        if groups:
            next_dest = groups[0]["destination"]
            try:
                sim_state.move_vehicle(vid, next_dest)
                print(f"moving loaded {vid} to {next_dest}")
            except Exception as e:
                print(f"failed move loaded {vid}: {e}")
        return

    waiting_here = _boxes_at_hub(boxes, loc)
    if not waiting_here:
        return

    region_groups = _group_by_destination_region(waiting_here, boxes)
    if not region_groups:
        return

    best_region = region_groups[0]
    region_center = best_region["center"]
    region_box_count = len(best_region["box_ids"])

    vehicle_type = vehicle["vehicle_type"]

    if vehicle_type == "CargoShip":
        transfer_hub = _choose_ship_transfer_target(loc, region_center, all_hub_centers)
        if transfer_hub is None:
            return

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, best_region["box_ids"])
        if loaded <= 0:
            return

        try:
            sim_state.move_vehicle(vid, transfer_hub)
            print(f"ship {vid} moving hub-to-hub {transfer_hub}")
        except Exception as e:
            print(f"failed ship move {vid}: {e}")
        return

    exact_groups = _group_by_exact_destination(waiting_here, boxes)
    if not exact_groups:
        return

    loaded = _load_up_to_capacity(sim_state, vid, vehicle, exact_groups[0]["box_ids"])
    if loaded <= 0:
        return

    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    vehicle = vehicles.get(vid)
    if vehicle is None or not vehicle["cargo"]:
        return

    next_dest = _group_by_exact_destination(vehicle["cargo"], boxes)[0]["destination"]
    try:
        sim_state.move_vehicle(vid, next_dest)
        print(f"ground/air {vid} moving to {next_dest}")
    except Exception as e:
        print(f"failed move {vid}: {e}")


def _spawn_for_waiting_hubs(sim_state):
    vehicles = sim_state.get_vehicles()
    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    hub_centers = [h["center"] for h in hubs]

    idle_locations = []
    for vehicle in vehicles.values():
        if vehicle["destination"] is None:
            idle_locations.append(vehicle["location"])

    for hub in hubs:
        hub_center = hub["center"]
        waiting_ids = _boxes_at_hub(sim_state.get_boxes(), hub_center)
        if not waiting_ids:
            continue

        already_served = False
        for loc in idle_locations:
            if _distance_m(loc, hub_center) <= _PROXIMITY_M:
                already_served = True
                break
        if already_served:
            continue

        region_groups = _group_by_destination_region(waiting_ids, sim_state.get_boxes())
        if not region_groups:
            continue

        best_region = region_groups[0]
        region_center = best_region["center"]
        region_box_count = len(best_region["box_ids"])

        nearest_dest_hub, _ = _find_nearest_hub(region_center, hub_centers, exclude=hub_center)

        prefer_ship = False
        planned_target = region_center

        if nearest_dest_hub is not None:
            if _distance_m(hub_center, region_center) > 300000 and region_box_count >= _SHIP_MIN_BOXES:
                prefer_ship = True
                planned_target = nearest_dest_hub

        vid = _choose_vehicle_for_leg(
            sim_state,
            hub_center,
            planned_target,
            region_box_count,
            prefer_ship=prefer_ship,
        )
        if vid is None:
            continue

        vehicles = sim_state.get_vehicles()
        vehicle = vehicles.get(vid)
        if vehicle is None:
            continue

        if prefer_ship:
            candidate_ids = best_region["box_ids"]
        else:
            exact_groups = _group_by_exact_destination(waiting_ids, sim_state.get_boxes())
            candidate_ids = exact_groups[0]["box_ids"] if exact_groups else waiting_ids

        loaded = _load_up_to_capacity(sim_state, vid, vehicle, candidate_ids)
        if loaded <= 0:
            continue

        try:
            sim_state.move_vehicle(vid, planned_target)
            print(f"new vehicle {vid} moving to {planned_target}")
        except Exception as e:
            print(f"failed move new vehicle {vid}: {e}")


def step(sim_state):
    print(f"tick={sim_state.tick} total_cost={sim_state.total_cost} terrain_penalty={getattr(sim_state, 'terrain_penalty', 'n/a')}")

    if sim_state.tick == 0:
        _spawn_initial(sim_state)
        return

    boxes = _boxes_waiting(sim_state.get_boxes())
    hubs = _cluster_hubs(boxes)
    all_hub_centers = [h["center"] for h in hubs]

    vehicles = sim_state.get_vehicles()
    for vid, vehicle in vehicles.items():
        if vehicle["destination"] is not None:
            continue
        _manage_idle_vehicle(sim_state, vid, vehicle, all_hub_centers)

    _spawn_for_waiting_hubs(sim_state)
