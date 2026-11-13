import pytest

from awi.core.layout import LayoutConfig

SMALL_LAYOUT = LayoutConfig(
    shelf_block_rows=2,
    shelf_segments=2,
    shelf_length=4,
    num_drop_stations=2,
    num_chargers=2,
)


@pytest.fixture
def small_layout_cfg() -> LayoutConfig:
    return SMALL_LAYOUT
