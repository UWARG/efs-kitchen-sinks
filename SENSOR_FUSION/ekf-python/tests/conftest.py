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
