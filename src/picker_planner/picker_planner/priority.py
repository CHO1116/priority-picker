"""피킹 순서 결정 로직 (ROS 의존성 없음 → pytest로 바로 테스트 가능).

우선순위 기준 (strategy="priority"):
  1. 업무: 긴급 주문에 필요한 색상 먼저
  2. 물리: 주변 간격(min_gap)이 그리퍼 손가락 여유보다 큰 것만
  3. 효율: (기준점 → 큐브) + (큐브 → 목적지 박스) 이동 거리가 짧은 것 먼저

비교 실험용 전략:
  - "color_order": 색상 고정 순서(config의 color_order)대로, 거리 무시
  - "random":      필요한 큐브 중 무작위
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

URGENT = 0
NORMAL = 1


@dataclass
class Cube:
    color: str
    x: float
    y: float
    z: float
    yaw: float = 0.0
    confidence: float = 1.0
    min_gap: float = 999.0


@dataclass
class OrderState:
    order_id: int
    priority: int
    box_id: str
    quantity: Dict[str, int]
    picked: Dict[str, int] = field(default_factory=dict)
    shortage: Dict[str, int] = field(default_factory=dict)  # 재고 부족으로 포기한 수량

    def remaining(self, color: str) -> int:
        return (self.quantity.get(color, 0)
                - self.picked.get(color, 0)
                - self.shortage.get(color, 0))

    def is_finished(self) -> bool:
        return all(self.remaining(c) <= 0 for c in self.quantity)

    @property
    def status(self) -> str:
        if not self.is_finished():
            return 'IN_PROGRESS' if any(self.picked.values()) else 'PENDING'
        return 'PARTIAL' if any(v > 0 for v in self.shortage.values()) else 'DONE'


@dataclass
class Selection:
    cube: Cube
    order: OrderState
    cost_mm: float
    gap_ignored: bool = False  # 틈이 부족하지만 다른 후보가 없어 시도하는 경우


def xy_dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sorted_open_orders(orders: List[OrderState]) -> List[OrderState]:
    return sorted((o for o in orders if not o.is_finished()),
                  key=lambda o: (o.priority, o.order_id))


def order_for_color(orders: List[OrderState], color: str) -> Optional[OrderState]:
    """이 색을 필요로 하는 주문 중 가장 우선순위가 높은 주문."""
    for o in sorted_open_orders(orders):
        if o.remaining(color) > 0:
            return o
    return None


def needed_colors(orders: List[OrderState]) -> Dict[str, int]:
    need: Dict[str, int] = {}
    for o in sorted_open_orders(orders):
        for c in o.quantity:
            if o.remaining(c) > 0:
                need[c] = need.get(c, 0) + o.remaining(c)
    return need


def find_shortages(cubes: List[Cube], orders: List[OrderState]) -> Dict[str, int]:
    """주문에 필요한데 작업대에 하나도 없는 색상 → {색: 부족 수량}."""
    present = {c.color for c in cubes}
    return {color: n for color, n in needed_colors(orders).items() if color not in present}


def select_next(
    cubes: List[Cube],
    orders: List[OrderState],
    ref_xy: Sequence[float],
    box_xy: Dict[str, Sequence[float]],
    strategy: str = 'priority',
    finger_clearance_mm: float = 15.0,
    min_confidence: float = 0.5,
    color_order: Sequence[str] = ('red', 'blue', 'green', 'yellow'),
    rng: Optional[random.Random] = None,
) -> Optional[Selection]:
    """다음에 집을 큐브 1개를 고른다. 집을 것이 없으면 None."""
    candidates: List[Tuple[tuple, Selection]] = []
    blocked: List[Tuple[tuple, Selection]] = []

    for cube in cubes:
        if cube.color == 'unknown' or cube.confidence < min_confidence:
            continue
        order = order_for_color(orders, cube.color)
        if order is None:
            continue  # 주문에 없는 색 → 나중에 보류함으로
        cost = xy_dist(ref_xy, (cube.x, cube.y)) + xy_dist((cube.x, cube.y), box_xy[order.box_id])
        sel = Selection(cube=cube, order=order, cost_mm=cost)

        if strategy == 'priority':
            key = (order.priority, cost)
        elif strategy == 'color_order':
            rank = color_order.index(cube.color) if cube.color in color_order else len(color_order)
            key = (rank, 0.0)
        elif strategy == 'random':
            key = (0, 0.0)
        else:
            raise ValueError(f'알 수 없는 strategy: {strategy}')

        if strategy == 'priority' and cube.min_gap < finger_clearance_mm:
            blocked.append((key, sel))
        else:
            candidates.append((key, sel))

    if strategy == 'random' and candidates:
        return (rng or random).choice(candidates)[1]

    if candidates:
        candidates.sort(key=lambda t: t[0])
        return candidates[0][1]

    if blocked:
        # 모든 후보가 붙어 있으면 무한 대기 대신, 가장 좋은 것을 시도한다.
        blocked.sort(key=lambda t: t[0])
        sel = blocked[0][1]
        sel.gap_ignored = True
        return sel
    return None


def leftover_cubes(cubes: List[Cube], orders: List[OrderState]) -> List[Cube]:
    """주문에 필요 없는 큐브 (보류함으로 옮길 대상)."""
    need = needed_colors(orders)
    return [c for c in cubes if c.color == 'unknown' or need.get(c.color, 0) <= 0]


def position_key(cube: Cube, grid_mm: float = 20.0) -> Tuple[str, int, int]:
    """재시도 횟수를 셀 때 '같은 큐브'로 보기 위한 키 (색 + 20mm 격자 위치)."""
    return (cube.color, round(cube.x / grid_mm), round(cube.y / grid_mm))
