from pathlib import Path

from blacknode.packages import load_package


def test_skills_layer_catalog_loads_with_components_disabled():
    info = load_package(Path(__file__).resolve().parents[1])
    assert info.ok
    assert info.layer == "skills"
    assert info.component_mode is True
    assert info.enabled_components == []
    assert set(info.components) == {"pick-place", "follow-person", "delivery", "docking", "inspection"}
