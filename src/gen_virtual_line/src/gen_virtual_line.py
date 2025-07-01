#!/usr/bin/env python3

#App
import rospy
import numpy as np
import cv2

#Manager Process
import sys
import threading
import multiprocessing
import signal
import os

#Plot
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.insert(0, os.path.dirname(__file__))

#Local
from play_process import PlayProcess
from detect_line import DetectLineNode
from simplify_points import simplify,save_image

# Dados compartilhados
bag_path = ""
file_path = ""
data = np.zeros((10, 2))
image = np.zeros((480, 640, 3), dtype=np.uint8)
data_lock = threading.Lock()
detectLine = None
process = None
after_id = None

signal.signal(signal.SIGINT, signal.SIG_DFL)

# Thread que atualiza os dados continuamente
def data_update_loop():
    global data
    global image
    global bag_path
    global file_path

    x_values = []
    y_values = []

    rate = rospy.Rate(100)  # 100 Hz

    print("⤳ Generate")
    process.run(bag_path)

    while not rospy.is_shutdown():
        process.out()
        # Update the scatter plot with the new data points
        if(len(detectLine.local_points) > 0 and len(detectLine.cv2Image) > 0):
            for i in detectLine.local_points:
                x_values.append(i[0])
                y_values.append(i[1])
            with data_lock:
                data = list(zip(x_values, y_values))
                image = detectLine.cv2Image

        if(process.finish):

            print("---------------")
            print("📈 Simplify")
            data_path = simplify(detectLine.path)
            data_path.append([[-4.2, 0.0],
                        [-4.2, 4.79],
                        [2.3, 4.79],
                        [2.3, 0],
                        [0.2, 0]])

            print("---------------")
            print("📂 Create struct folder")
            script_dir = os.path.dirname(os.path.abspath(__file__))
            data_dir = os.path.join(script_dir,  "data_result")
            os.makedirs(data_dir, exist_ok=True)
            image_dir = os.path.join(script_dir,  "graph_result")
            os.makedirs(image_dir, exist_ok=True)
            bag_path = os.path.basename(os.path.normpath(bag_path))
            bag_name = bag_path.replace(".bag", '')
            save_dir = os.path.join(image_dir, bag_name + "_graph.png")
            file_path = os.path.join(data_dir, bag_name + "_result.csv")

            print("---------------")
            print("📷 Save image")
            save_process(data_path, save_dir)
            print("---------------")
            print("⬇ Save points")
            np.savetxt(file_path, data_path[1], delimiter = ",") #Save points
            print("---------------")
            print("🏁 Finish!")
            break
        rate.sleep()
    root.after(0, shutdown_gui)


def save_process(data, save_dir):
    p = multiprocessing.Process(target=save_image, args=(data, save_dir))
    p.start()

# Função de atualização visual (roda no mainloop do Tkinter)
def update_plot():
    global after_id
    with data_lock:
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_plot.set_data(rgb_image)
        scatter_plot.set_offsets(data)
        xdata = [i[0] for i in data]
        ydata = [i[1] for i in data]
        margin = 1.0
        xmin, xmax = min(xdata), max(xdata)
        ymin, ymax = min(ydata), max(ydata)
        ax_scatter.set_xlim(xmin - margin, xmax + margin)
        ax_scatter.set_ylim(ymin - margin, ymax + margin)

    canvas.draw()
    after_id = root.after(100, update_plot)  # Atualiza a cada 100ms


def shutdown_gui():
    global after_id
    if after_id is not None:
        root.after_cancel(after_id)
    root.quit()
    root.destroy()


if __name__ == "__main__":
    multiprocessing.set_start_method("spawn", force=True)

    bag_path = sys.argv[1]
    detectLine = DetectLineNode()
    process = PlayProcess()

    root = tk.Tk()
    root.title("Imagem + Scatter com Matplotlib")

    # Figura com dois subplots: imagem e scatter
    fig, (ax_img, ax_scatter) = plt.subplots(1, 2, figsize=(10, 5))

    # Subplot da imagem
    img_plot = ax_img.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
    ax_img.axis("off")
    ax_img.set_title("Robot Image")

    # Subplot do scatter
    scatter_plot = ax_scatter.scatter([], [])
    ax_scatter.set_title("Detect Line")
    ax_scatter.set_xlabel("x(m)")
    ax_scatter.set_ylabel("y(m)")
    ax_scatter.grid()

    # Embutir figura no Tkinter
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # Inicia a thread de dados
    threading.Thread(target=data_update_loop, daemon=True).start()

    # Inicia a primeira atualização
    update_plot()

    # Inicia a interface gráfica
    root.mainloop()