import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    df = pd.read_csv("ss_data.csv")
    
    time = df["time"].to_numpy()
    p = df["p"].to_numpy()
    q = df["q"].to_numpy()
    r = df["r"].to_numpy()
    phi = df["phi"].to_numpy()
    theta = df["theta"].to_numpy()
    psi = df["psi"].to_numpy()
    delta1 = df["delta1"].to_numpy()
    delta2 = df["delta2"].to_numpy()
    delta3 = df["delta3"].to_numpy()
    delta4 = df["delta4"].to_numpy()
    

    fig, axs = plt.subplots(5, 2, figsize=(10, 12))

    axs[0, 0].plot(time, p)
    axs[0, 0].set_ylabel("p")

    axs[1, 0].plot(time, q)
    axs[1, 0].set_ylabel("q")

    axs[2, 0].plot(time, r)
    axs[2, 0].set_ylabel("r")

    axs[3, 0].plot(time, phi)
    axs[3, 0].set_ylabel("phi")

    axs[0, 1].plot(time, theta)
    axs[0, 1].set_ylabel("theta")

    axs[1, 1].plot(time, psi)
    axs[1, 1].set_ylabel("psi")

    axs[2, 1].plot(time, delta1)
    axs[2, 1].set_ylabel("delta1")

    axs[3, 1].plot(time, delta2)
    axs[3, 1].set_ylabel("delta2")

    axs[4, 0].plot(time, delta3)
    axs[4, 0].set_ylabel("delta3")

    axs[4, 1].plot(time, delta4)
    axs[4, 1].set_ylabel("delta4")

    for i in range(4):
        for j in range(2):
            axs[i, j].set_xlabel("time")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
