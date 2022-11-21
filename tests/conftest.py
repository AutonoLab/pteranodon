import pytest


def pytest_addoption(parser: pytest.Parser):
    parser.addoption(
        "--hostname",
        action="store",
        default="",
        help="Hostname for MAV SDK server, default is none",
    )
    parser.addoption(
        "--port",
        action="store",
        default=14540,
        help="Port for MAV SDK server, default is 14540",
    )


@pytest.fixture(scope="session")
def hostname(request: pytest.FixtureRequest):
    return request.config.getoption("--hostname")


@pytest.fixture(scope="session")
def port(request: pytest.FixtureRequest):
    return request.config.getoption("--port")
