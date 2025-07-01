import numpy as np
from rdp import rdp  # Ramer–Douglas–Peucker

def simplify(data):

    # Get data from file
    points = np.array([[i[0], i[1]] for i in data])

    #RDP fitting
    simplified = rdp(points, epsilon=0.3)

    #RDP fitting - Simplify to 90° corners
    last_axe = 0
    for i in range(1, len(simplified)):
        vec = simplified[i] - simplified[i-1]
        angle = np.degrees(np.arctan2(vec[1], vec[0])) % 360

        axe_close = 0
        for axe in range(0, 361, 90):
            if(np.isclose(angle, axe, atol=45)):  axe_close = axe

        mod = np.sqrt(np.power(vec[1], 2) + np.power(vec[0], 2))

        if(np.isclose(angle, axe_close, atol=2)):
            last_axe = axe_close
            continue

        angle = np.deg2rad((angle - axe_close))

        if(last_axe == 0 or last_axe == 360) or (last_axe == 180) :
            simplified[i-1, 0] = simplified[i, 0]
        else:
            simplified[i-1, 1] = simplified[i, 1]

        last_axe = axe_close

    #Dense Path with points with 0.2 of distance
    path_points = []
    for i in range(0, len(simplified) - 1):
        vec = simplified[i+1] - simplified[i]
        mod = np.sqrt(np.power(vec[1], 2) + np.power(vec[0], 2))

        number = round(mod/0.2)

        x_vals = np.linspace(simplified[i, 0], simplified[i + 1, 0],  number)
        y_vals = np.linspace(simplified[i, 1], simplified[i + 1, 1],  number)
        path_points.extend([[x_vals[j], y_vals[j]] for j in range(len(x_vals))])

    path_points = np.array(path_points)

    return [simplified, path_points]

def save_image(data, save_dir):
    import matplotlib
    matplotlib.use("Agg")  # ⬅ Backend não interativo (NÃO usa Tkinter)
    import matplotlib.pyplot as plt
    import numpy as np
    simplified = data[0]
    path_points = data[1]
    pointsGT = data[2]

    fig, ax = plt.subplots()

    #Ground Truth
    points = np.array([[i[0], i[1]] for i in pointsGT])
    ax.plot(points[:, 0], points[:, 1], 'g', linewidth=5, label="Ground Truth")

    #Simplificado
    ax.plot(simplified[0, 0], simplified[0, 1], 'ro', markersize=15)

    #Simplificado condensado
    ax.plot(path_points[:, 0], path_points[:, 1], 'bo', markersize=10, label="RDP Aproximate")

    ax.legend()
    ax.set_title("Path Virtual basing in RDP Aproximation", fontsize=24)
    ax.grid()
    plt.axis("equal")
    ax.set_xlabel("x (m)", fontsize=18)
    ax.set_ylabel("y (m)", fontsize=18)

    # Salva o gráfico como imagem temporária
    fig.savefig(save_dir)
