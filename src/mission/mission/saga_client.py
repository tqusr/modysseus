import requests


class SagaApi:
    """Read-only client for the Saga AI event server.

    The truck never reports anything back — route status is advanced by
    organizers pressing buttons on their own admin page. Every method here
    is a GET.
    """

    def __init__(self, server_url: str, token: str, timeout: float = 0.5):
        self.server_url = server_url.rstrip('/')
        self.headers = {'Authorization': f'Bearer {token}'}
        self.timeout = timeout

    def _get(self, path: str) -> dict:
        response = requests.get(
            self.server_url + path,
            headers=self.headers,
            timeout=self.timeout,
        )
        response.raise_for_status()
        return response.json()

    def get_route(self):
        """The truck's current route, or None if the event hasn't started."""
        return self._get('/api/v1/route')['route']

    def get_status(self) -> dict:
        return self._get('/api/v1/status')

    def get_route_by_id(self, route_id: int) -> dict:
        return self._get(f'/api/v1/routes/{route_id}')

    def get_docks(self) -> list:
        return self._get('/api/v1/docks')
