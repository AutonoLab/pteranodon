import pytest


def pytest_addoption(parser : pytest.Parser):
    parser.addoption(
        "--hostname", action="store", default="", help="Hostname for MAV SDK server, default is none"
    )


@pytest.fixture
def hostname(request : pytest.FixtureRequest):
    return request.config.getoption("--hostname")
