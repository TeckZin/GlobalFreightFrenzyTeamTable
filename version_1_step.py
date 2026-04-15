from simulator import VehicleType, haversine_distance_meters
import random

SEED = 42
random.seed(SEED)

_PROXIMITY_M = 50.0

VEHICLE_ENUM = {
    "SemiTruck": VehicleType.SemiTruck,
    "Train": VehicleType.Train,
    "Airplane": VehicleType.Airplane,
    "CargoShip": VehicleType.CargoShip,
    "Drone": VehicleType.Drone,
}

VEHICLE_STATS = {
    "SemiTruck": {
        "base_cost": 100.0,
        "per_km_cost": 0.05,
        "speed_kmh": 80.0,
        "capacity": 50,
        "mode": "land",
    },
    "Train": {
        "base_cost": 500.0,
        "per_km_cost": 0.02,
        "speed_kmh": 120.0,
        "capacity": 500,
        "mode": "land",
    },
    "Airplane": {
        "base_cost": 2000.0,
        "per_km_cost": 0.50,
        "speed_kmh": 800.0,
        "capacity": 100,
        "mode": "air",
    },
    "CargoShip": {
        "base_cost": 1000.0,
        "per_km_cost": 0.01,
        "speed_kmh": 30.0,
        "capacity": 1000,
        "mode": "ocean",
    },
    "Drone": {
        "base_cost": 50.0,
        "per_km_cost": 0.30,
        "speed_kmh": 50.0,
        "capacity": 5,
        "mode": "air",
    },
}

DISPATCH_STATE = {
    "vehicle_plans": {},
    "hub_vehicles": {},
}


class Hub:
    def __init__(self, location):
        self.location = location
        self.cargo_count = 0
        self.destination_counts = {}
        self.destination_distances_km = {}
        self.destination_boxes = {}
        self.boxes = []
        self.score = 0.0
        self.score_details = {}

    def add_box(self, box):
        destination = box["destination"]
        self.cargo_count += 1
        self.destination_counts[destination] = self.destination_counts.get(destination, 0) + 1
        dist_km = haversine_distance_meters(self.location, destination) / 1000.0
        self.destination_distances_km[destination] = dist_km
        self.destination_boxes.setdefault(destination, []).append(box)
        self.boxes.append(box)

    def __repr__(self):
        return (
            f"Hub(location={self.location}, "
            f"cargo_count={self.cargo_count}, "
            f"destination_counts={self.destination_counts}, "
            f"destination_distances_km={self.destination_distances_km}, "
            f"score={self.score}, "
            f"score_details={self.score_details})"
        )


def distance_km(a, b):
    return haversine_distance_meters(a, b) / 1000.0


def distance_m(a, b):
    return haversine_distance_meters(a, b)


def route_crosses_forbidden_terrain(start, end, vehicle_name):
    mode = VEHICLE_STATS[vehicle_name]["mode"]

    if mode == "air":
        return False

    a_lat, a_lon = start
    b_lat, b_lon = end

    if mode == "land":
        if abs(a_lon - b_lon) > 20 or abs(a_lat - b_lat) > 20:
            return True

    if mode == "ocean":
        if abs(a_lon - b_lon) < 3 and abs(a_lat - b_lat) < 3:
            return True

    return False


def is_route_allowed(start, end, vehicle_name):
    return not route_crosses_forbidden_terrain(start, end, vehicle_name)

def movement_cost_km(distance_km_value, vehicle_name):
    stats = VEHICLE_STATS[vehicle_name]
    return stats["per_km_cost"] * distance_km_value


def spawn_plus_move_cost(start, end, vehicle_name, include_base_cost=True):
    stats = VEHICLE_STATS[vehicle_name]
    d_km = distance_km(start, end)
    cost = movement_cost_km(d_km, vehicle_name)
    if include_base_cost:
        cost += stats["base_cost"]
    return cost


def delivery_leg_cost(origin, destination, vehicle_name, box_count=1, include_load_cost=True):
    d_km = distance_km(origin, destination)
    move_cost = movement_cost_km(d_km, vehicle_name)
    load_cost = float(box_count) if include_load_cost else 0.0
    return move_cost + load_cost


def cargo_value_simple(box_count, distance_km_total, distance_weight=1.0):
    return float(box_count) + (distance_weight * distance_km_total)


def goods_per_cost_score(goods_value, estimated_cost, epsilon=1e-9):
    return goods_value / max(estimated_cost, epsilon)


def box_trip_value(box, pickup_location=None, distance_weight=1.0):
    origin = pickup_location if pickup_location is not None else box["location"]
    d_km = distance_km(origin, box["destination"])
    return cargo_value_simple(box_count=1, distance_km_total=d_km, distance_weight=distance_weight)


def aggregate_box_value(boxes, pickup_location=None, distance_weight=1.0):
    total = 0.0
    for box in boxes:
        total += box_trip_value(
            box,
            pickup_location=pickup_location,
            distance_weight=distance_weight,
        )
    return total


def estimate_event_penalty(vehicle_name, active_events, pickup_location=None, destination=None):
    mode = VEHICLE_STATS[vehicle_name]["mode"]
    penalty = 0.0

    for event in active_events:
        event_type = event.get("type")

        if event_type == "ground_stop_flights" and mode == "air":
            penalty += 1000000.0
        elif event_type == "traffic" and mode == "land":
            speed_multiplier = event.get("speed_multiplier", 0.25)
            slowdown = max(0.0, 1.0 - speed_multiplier)
            penalty += 1000.0 * slowdown
        elif event_type == "oceanic_weather" and mode == "ocean":
            penalty += 1000000.0

    return penalty


def estimate_capacity_penalty(vehicle_name, box_count):
    capacity = VEHICLE_STATS[vehicle_name]["capacity"]
    if box_count <= capacity:
        return 0.0

    overflow = box_count - capacity
    return 10000.0 + (1000.0 * overflow)


def candidate_boxes_for_vehicle(boxes, vehicle_name):
    capacity = VEHICLE_STATS[vehicle_name]["capacity"]
    return boxes[:capacity]


def score_route_details(
    vehicle_start,
    pickup_location,
    destination,
    candidate_boxes,
    vehicle_name,
    active_events=None,
    include_base_cost=True,
    distance_weight=1.0,
):
    active_events = active_events or []

    if not is_route_allowed(vehicle_start, pickup_location, vehicle_name):
        return None

    if not is_route_allowed(pickup_location, destination, vehicle_name):
        return None

    chosen_boxes = candidate_boxes_for_vehicle(candidate_boxes, vehicle_name)
    box_count = len(chosen_boxes)

    if box_count == 0:
        return None

    goods_value = aggregate_box_value(
        chosen_boxes,
        pickup_location=pickup_location,
        distance_weight=distance_weight,
    )

    event_penalty = estimate_event_penalty(
        vehicle_name,
        active_events,
        pickup_location=pickup_location,
        destination=destination,
    )

    capacity_penalty = estimate_capacity_penalty(vehicle_name, box_count)

    base_visit_cost = spawn_plus_move_cost(
        vehicle_start,
        pickup_location,
        vehicle_name,
        include_base_cost=include_base_cost,
    )

    delivery_cost = delivery_leg_cost(
        pickup_location,
        destination,
        vehicle_name,
        box_count=box_count,
        include_load_cost=True,
    )

    total_cost = base_visit_cost + delivery_cost + event_penalty + capacity_penalty
    score = goods_per_cost_score(goods_value, total_cost)

    return {
        "vehicle": vehicle_name,
        "pickup_location": pickup_location,
        "destination": destination,
        "box_count": box_count,
        "box_ids": [box["id"] for box in chosen_boxes],
        "goods_value": goods_value,
        "base_visit_cost": base_visit_cost,
        "delivery_cost": delivery_cost,
        "event_penalty": event_penalty,
        "capacity_penalty": capacity_penalty,
        "total_cost": total_cost,
        "score": score,
    }


def best_route_for_hub(hub, active_events=None):
    active_events = active_events or []
    best = None

    for destination, dest_boxes in hub.destination_boxes.items():
        available_boxes = [b for b in dest_boxes if not b["delivered"] and b["vehicle_id"] is None]

        if not available_boxes:
            continue

        for vehicle_name in VEHICLE_STATS:
            details = score_route_details(
                vehicle_start=hub.location,
                pickup_location=hub.location,
                destination=destination,
                candidate_boxes=available_boxes,
                vehicle_name=vehicle_name,
                active_events=active_events,
                include_base_cost=True,
            )

            if details is None:
                continue

            if best is None or details["score"] > best["score"]:
                best = details

    return best


def build_hubs(boxes):
    hubs = {}

    for box in boxes.values():
        if box["delivered"]:
            continue

        location = box["location"]

        if location not in hubs:
            hubs[location] = Hub(location)

        hubs[location].add_box(box)

    return hubs


def refresh_hub_scores(hubs, active_events):
    for hub in hubs.values():
        best = best_route_for_hub(hub, active_events=active_events)
        if best is None:
            hub.score = 0.0
            hub.score_details = {}
        else:
            hub.score = best["score"]
            hub.score_details = best



def try_create_best_vehicle(sim_state, hub, active_events):
    best = best_route_for_hub(hub, active_events)
    if best is None:
        return None

    preferred_vehicle = best["vehicle"]
    vehicle_order = [preferred_vehicle] + [v for v in VEHICLE_STATS if v != preferred_vehicle]

    for vehicle_name in vehicle_order:
        route = best_route_for_hub(hub, active_events)
        if route is None:
            return None

        if route["vehicle"] != vehicle_name:
            continue

        if not is_route_allowed(hub.location, hub.location, vehicle_name):
            continue

        if not is_route_allowed(hub.location, route["destination"], vehicle_name):
            continue

        try:
            vehicle_id = sim_state.create_vehicle(VEHICLE_ENUM[vehicle_name], hub.location)
            DISPATCH_STATE["vehicle_plans"][vehicle_id] = {
                "phase": "to_pickup",
                "hub_location": hub.location,
                "pickup_location": hub.location,
                "destination": route["destination"],
                "vehicle_name": vehicle_name,
                "box_ids": route["box_ids"],
            }
            DISPATCH_STATE["hub_vehicles"][hub.location] = vehicle_id
            sim_state.move_vehicle(vehicle_id, hub.location)
            return vehicle_id
        except ValueError:
            continue

    return None

def assign_new_plan_to_existing_vehicle(sim_state, vehicle_id, hub, active_events):
    best = best_route_for_hub(hub, active_events)
    if best is None:
        return False

    DISPATCH_STATE["vehicle_plans"][vehicle_id] = {
        "phase": "to_pickup",
        "hub_location": hub.location,
        "pickup_location": hub.location,
        "destination": best["destination"],
        "vehicle_name": best["vehicle"],
        "box_ids": best["box_ids"],
    }
    sim_state.move_vehicle(vehicle_id, hub.location)
    return True


def box_ids_still_available(boxes, box_ids, location=None, destination=None):
    valid = []
    for box_id in box_ids:
        if box_id not in boxes:
            continue
        box = boxes[box_id]
        if box["delivered"]:
            continue
        if box["vehicle_id"] is not None:
            continue
        if location is not None and box["location"] != location:
            continue
        if destination is not None and box["destination"] != destination:
            continue
        valid.append(box_id)
    return valid


def at_location(a, b, threshold_m=_PROXIMITY_M):
    return distance_m(a, b) <= threshold_m


def process_vehicle_plan(sim_state, vehicle_id, vehicle, boxes):
    plan = DISPATCH_STATE["vehicle_plans"].get(vehicle_id)
    if plan is None:
        return

    current_location = vehicle["location"]
    pickup_location = plan["pickup_location"]
    destination = plan["destination"]

    if plan["phase"] == "to_pickup":
        if at_location(current_location, pickup_location):
            available_box_ids = box_ids_still_available(
                boxes,
                plan["box_ids"],
                location=pickup_location,
                destination=destination,
            )

            if available_box_ids:
                try:
                    sim_state.load_vehicle(vehicle_id, available_box_ids)
                except ValueError:
                    smaller = available_box_ids[: max(1, len(available_box_ids) // 2)]
                    if smaller:
                        try:
                            sim_state.load_vehicle(vehicle_id, smaller)
                            available_box_ids = smaller
                        except ValueError:
                            available_box_ids = []

            if available_box_ids:
                plan["box_ids"] = available_box_ids
                plan["phase"] = "to_dropoff"
                sim_state.move_vehicle(vehicle_id, destination)
            else:
                plan["phase"] = "idle"
        else:
            sim_state.move_vehicle(vehicle_id, pickup_location)

    elif plan["phase"] == "to_dropoff":
        if not is_route_allowed(pickup_location, destination, plan["vehicle_name"]):
            plan["phase"] = "idle"
            return

        if at_location(current_location, destination):
            cargo_ids = list(vehicle["cargo"])
            if cargo_ids:
                try:
                    sim_state.unload_vehicle(vehicle_id, cargo_ids)
                except ValueError:
                    pass
            plan["phase"] = "idle"
        else:
            sim_state.move_vehicle(vehicle_id, destination)

def step(sim_state):
    tick = sim_state.tick
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    try:
        active_events = sim_state.get_active_events()
        if not active_events:
            print("get_active_events exists but returned empty list")
    except AttributeError:
        active_events = []
        print("get_active_events endpoint does not exist")

    hubs = build_hubs(boxes)
    refresh_hub_scores(hubs, active_events)

    for vehicle_id, vehicle in vehicles.items():
        process_vehicle_plan(sim_state, vehicle_id, vehicle, boxes)

    vehicles = sim_state.get_vehicles()

    sorted_hubs = sorted(
        hubs.values(),
        key=lambda h: h.score,
        reverse=True,
    )

    for hub in sorted_hubs:
        if hub.cargo_count <= 0:
            continue

        assigned_vehicle_id = DISPATCH_STATE["hub_vehicles"].get(hub.location)

        if assigned_vehicle_id is None:
            try_create_best_vehicle(sim_state, hub, active_events)
            continue

        if assigned_vehicle_id not in vehicles:
            DISPATCH_STATE["hub_vehicles"].pop(hub.location, None)
            DISPATCH_STATE["vehicle_plans"].pop(assigned_vehicle_id, None)
            try_create_best_vehicle(sim_state, hub, active_events)
            continue

        vehicle = vehicles[assigned_vehicle_id]
        plan = DISPATCH_STATE["vehicle_plans"].get(assigned_vehicle_id)

        if plan is None or plan.get("phase") == "idle":
            assign_new_plan_to_existing_vehicle(sim_state, assigned_vehicle_id, hub, active_events)

    if tick == 0:
        print("=== ACTIVE EVENTS ===")
        print(active_events)
        print("=== HUBS ===")
        for location, hub in hubs.items():
            print(location, hub)
