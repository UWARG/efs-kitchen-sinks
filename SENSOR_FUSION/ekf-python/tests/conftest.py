import pytest
from pathlib import Path

@pytest.fixture
def home_dir():
    return str(Path(__file__).resolve().parent.parent)

@pytest.fixture
def plots_dir():
    plots_path = Path(__file__).resolve().parent.parent / "plots"
    plots_path.mkdir(exist_ok=True)
    return str(plots_path)

def pytest_addoption(parser):
    parser.addoption(
        "--run-mission-planner", 
        action="store_true", 
        default=False, 
        help="run tests that require Mission Planner connection"
    )

def pytest_collection_modifyitems(config, items):
    if config.getoption("--run-mission-planner"):
        # --run-mission-planner given in cli: do not skip
        return
    
    skip_mp = pytest.mark.skip(reason="need --run-mission-planner option to run")
    for item in items:
        if "mission_planner" in item.keywords:
            item.add_marker(skip_mp)
