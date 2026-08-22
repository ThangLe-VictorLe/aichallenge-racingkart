import numpy as np

def line(x0, y0, x1, y1):
    """Bresenham's Line Algorithm. Returns (x_list, y_list) arrays."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    
    x_list = []
    y_list = []
    
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            x_list.append(x)
            y_list.append(y)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        x_list.append(x)
        y_list.append(y)
    else:
        err = dy / 2.0
        while y != y1:
            x_list.append(x)
            y_list.append(y)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        x_list.append(x)
        y_list.append(y)
        
    return np.array(x_list), np.array(y_list)
