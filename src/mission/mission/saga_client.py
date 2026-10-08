"""Read-only HTTP client for the Saga AI event server."""
import requests


class SagaApi:
    """Read-only client for the Saga AI event server.

    The truck never reports back. Organizers advance route status from their
    own admin page, so every method here is a GET.

    Attributes:
        server_url: Server base URL without a trailing slash.
        headers: Request headers carrying the bearer token.
        timeout: Per-request timeout in seconds.
    """

    def __init__(self, server_url: str, token: str, timeout: float = 0.5):
        """Initializes the client.

        Args:
            server_url: Server base URL.
            token: Bearer token for the Authorization header.
            timeout: Per-request timeout in seconds.
        """
        self.server_url = server_url.rstrip('/')
        self.headers = {'Authorization': f'Bearer {token}'}
        self.timeout = timeout

    def _get(self, path: str) -> dict:
        """Returns the parsed JSON body of a GET to `path`.

        Raises:
            requests.HTTPError: If the server returns an error status.
        """
        response = requests.get(
            self.server_url + path,
            headers=self.headers,
            timeout=self.timeout,
        )
        response.raise_for_status()
        return response.json()

    def get_route(self):
        """Returns the truck's current route, or None before the event starts."""
        return self._get('/api/v1/route')['route']

    def get_status(self) -> dict:
        """Returns the event status."""
        return self._get('/api/v1/status')

    def get_route_by_id(self, route_id: int) -> dict:
        """Returns the route with the given id."""
        return self._get(f'/api/v1/routes/{route_id}')

    def get_docks(self) -> list:
        """Returns all docks known to the server."""
        return self._get('/api/v1/docks')
