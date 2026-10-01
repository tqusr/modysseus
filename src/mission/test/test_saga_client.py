from unittest.mock import MagicMock, patch

import pytest
import requests

from mission.saga_client import SagaApi


def _fake_response(json_body, status_ok=True):
    response = MagicMock()
    response.json.return_value = json_body
    if status_ok:
        response.raise_for_status.return_value = None
    else:
        response.raise_for_status.side_effect = requests.HTTPError("401")
    return response


@patch("mission.saga_client.requests.get")
def test_get_route_returns_the_route_field(mock_get):
    mock_get.return_value = _fake_response({
        "truck_id": 1,
        "event_state": "running",
        "route": {"route_id": 5, "source": {"letter": "C", "tag_id": 2},
                   "destination": {"letter": "F", "tag_id": 5}},
    })

    api = SagaApi("http://localhost:8000", "secret-token")
    route = api.get_route()

    assert route["route_id"] == 5
    mock_get.assert_called_once_with(
        "http://localhost:8000/api/v1/route",
        headers={"Authorization": "Bearer secret-token"},
        timeout=0.5,
    )


@patch("mission.saga_client.requests.get")
def test_get_route_returns_none_before_event_starts(mock_get):
    mock_get.return_value = _fake_response({"truck_id": 1, "event_state": "not_started", "route": None})

    api = SagaApi("http://localhost:8000", "secret-token")

    assert api.get_route() is None


@patch("mission.saga_client.requests.get")
def test_wrong_token_raises(mock_get):
    mock_get.return_value = _fake_response({}, status_ok=False)

    api = SagaApi("http://localhost:8000", "wrong-token")

    with pytest.raises(requests.HTTPError):
        api.get_route()


@patch("mission.saga_client.requests.get")
def test_server_url_trailing_slash_is_stripped(mock_get):
    mock_get.return_value = _fake_response({"truck_id": 1, "event_state": "running", "route": None})

    api = SagaApi("http://localhost:8000/", "secret-token")
    api.get_route()

    mock_get.assert_called_once_with(
        "http://localhost:8000/api/v1/route",
        headers={"Authorization": "Bearer secret-token"},
        timeout=0.5,
    )
