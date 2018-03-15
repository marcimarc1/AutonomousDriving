import numpy as np

def grad(x, nth=1, n=2, h=0.1):
    ds = [{
        'grad': x,
        'grad_mag': x
    }]
    for i in range(nth):
        _x = ds[-1]['grad_mag']
        grad = np.array(np.gradient(_x, h))
        grad_mag = np.array(grad)
        # print(grad.shape)
        if (n > 1):
            grad_mag = np.linalg.norm(grad, axis=0)
        ds.append({
            'grad': grad,
            'grad_mag': grad_mag
        })
    return ds[-1]['grad_mag'], ds[-1]['grad'], ds

def euclid_dist(a, b, axis=1):
    if(np.all(a.shape == b.shape)):
        dist = np.linalg.norm(a-b, axis=axis)
    else:
        B = b[:, np.newaxis]
        sub = B-a
        dists = np.linalg.norm(sub, axis=2)
        dist = np.amin(dists,axis=0).T
    return dist

def cons_left_right(x, l_left, l_right):
    left = np.array(x)
    right = np.array(x)
    left[:, 1] = l_left
    right[:, 1] = l_right
    return left, right

def psi(x):
    d_x, _, _ = grad(x[:, 0], nth=1, n=1)
    d_y, _, _ = grad(x[:, 1], nth=1, n=1)
    return np.arctan2(d_y, d_x)

def kappa(x):
    _, _, dxs = grad(x[:, 0], 2, n=1)
    _, _, dys = grad(x[:, 1], 2, n=1)
    den = np.power(np.add(np.power(dxs[1]['grad'], 2), np.power(dys[1]['grad'], 2)), 1 / 3) + 1e-20
    den[den == 0.] = 1.
    num = np.subtract(np.multiply(dxs[1]['grad'], dys[2]['grad']), np.multiply(dys[1]['grad_mag'], dxs[2]['grad_mag']))
    return np.divide(num, den)

def normalize(x):
    return ((x - x.min()) / (x.max() - x.min()))

def normalize_std(x):
    return (x - np.mean(x)) / np.std(x)

def reshaper(x):
    return np.reshape(x, (len(x) // 2, 2))
