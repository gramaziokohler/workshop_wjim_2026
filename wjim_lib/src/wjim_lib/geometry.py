from typing import List

from compas.geometry import Vector
from compas_brep import Brep


# mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
def brep_edge_indices_along(brep: Brep, direction: Vector) -> List[int]:
    """Returns the indices of the edges of a Brep which are parallel to a direction.

    The order of the edges of a Brep depends on its backend, so they cannot be selected by a fixed index.

    Parameters
    ----------
    brep :
        The Brep.
    direction :
        The direction.

    Returns
    -------
        The indices of the edges.

    """
    direction = direction.unitized()
    indices = []
    for index, edge in enumerate(brep.edges):
        start, end = edge.vertices
        if abs(Vector.from_start_end(start.point, end.point).unitized().dot(direction)) > 0.999:
            indices.append(index)
    return indices
