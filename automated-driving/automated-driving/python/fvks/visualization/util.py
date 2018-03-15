from matplotlib.path import Path
import matplotlib.patches as patches


def draw_polygon_as_patch(vertices, ax, zorder=5, facecolor='#ffffff',
                          edgecolor='#000000', lw=0.5):
    """
    vertices are no closed polygon (first element != last element)
    """
    verts = []
    codes = [Path.MOVETO]
    for p in vertices:
        verts.append(p)
        codes.append(Path.LINETO)
    del codes[-1]
    codes.append(Path.CLOSEPOLY)
    verts.append((0, 0))

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=facecolor, edgecolor=edgecolor,
                              lw=lw, zorder=zorder)
    ax.add_patch(patch)