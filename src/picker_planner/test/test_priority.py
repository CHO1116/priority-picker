"""우선순위 로직 단위 테스트 (ROS 없이 실행 가능: python -m pytest src/picker_planner/test)."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from picker_planner.priority import (NORMAL, URGENT, Cube, OrderState,  # noqa: E402
                                     find_shortages, leftover_cubes, select_next)

BOX = {'A': (0.0, 400.0), 'B': (0.0, -400.0), 'HOLD': (-200.0, 0.0)}
SCAN = (300.0, 0.0)


def make_orders():
    return [
        OrderState(order_id=2, priority=NORMAL, box_id='B', quantity={'green': 1}),
        OrderState(order_id=1, priority=URGENT, box_id='A', quantity={'red': 1}),
    ]


def test_urgent_order_first_even_if_farther():
    cubes = [Cube('green', 300, 0, 0), Cube('red', 600, 300, 0)]
    sel = select_next(cubes, make_orders(), SCAN, BOX)
    assert sel.cube.color == 'red'
    assert sel.order.order_id == 1


def test_nearest_within_same_priority():
    orders = [OrderState(1, URGENT, 'A', {'red': 2})]
    near, far = Cube('red', 300, 350, 0), Cube('red', 600, -300, 0)
    sel = select_next([far, near], orders, SCAN, BOX)
    assert sel.cube is near


def test_blocked_cube_skipped_when_alternative_exists():
    orders = [OrderState(1, URGENT, 'A', {'red': 1})]
    blocked = Cube('red', 300, 350, 0, min_gap=5)
    free = Cube('red', 600, 0, 0, min_gap=50)
    sel = select_next([blocked, free], orders, SCAN, BOX, finger_clearance_mm=15)
    assert sel.cube is free and not sel.gap_ignored


def test_blocked_cube_tried_when_no_alternative():
    orders = [OrderState(1, URGENT, 'A', {'red': 1})]
    sel = select_next([Cube('red', 300, 350, 0, min_gap=5)], orders, SCAN, BOX)
    assert sel is not None and sel.gap_ignored


def test_unneeded_and_unknown_ignored():
    orders = [OrderState(1, URGENT, 'A', {'red': 1})]
    cubes = [Cube('blue', 300, 0, 0), Cube('unknown', 310, 0, 0), Cube('red', 0, 0, 0, confidence=0.1)]
    assert select_next(cubes, orders, SCAN, BOX) is None


def test_shortage_and_status():
    orders = [OrderState(1, URGENT, 'A', {'red': 2, 'blue': 1})]
    cubes = [Cube('red', 300, 0, 0)]
    assert find_shortages(cubes, orders) == {'blue': 1}
    orders[0].shortage['blue'] = 1
    orders[0].picked['red'] = 2
    assert orders[0].is_finished() and orders[0].status == 'PARTIAL'


def test_leftovers():
    orders = [OrderState(1, URGENT, 'A', {'red': 1})]
    cubes = [Cube('red', 0, 0, 0), Cube('blue', 0, 0, 0), Cube('unknown', 0, 0, 0)]
    assert [c.color for c in leftover_cubes(cubes, orders)] == ['blue', 'unknown']


def test_color_order_strategy_ignores_urgency():
    cubes = [Cube('green', 300, 0, 0), Cube('red', 600, 300, 0)]
    sel = select_next(cubes, make_orders(), SCAN, BOX, strategy='color_order',
                      color_order=['green', 'red'])
    assert sel.cube.color == 'green'


def test_random_strategy_returns_needed_cube():
    cubes = [Cube('green', 300, 0, 0), Cube('red', 600, 300, 0), Cube('blue', 0, 0, 0)]
    sel = select_next(cubes, make_orders(), SCAN, BOX, strategy='random', rng=random.Random(0))
    assert sel.cube.color in ('green', 'red')
