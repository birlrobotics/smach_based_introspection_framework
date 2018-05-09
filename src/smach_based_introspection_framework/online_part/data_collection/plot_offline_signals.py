if __name__ == "__main__":
    import pandas as pd
    import numpy as np
    import matplotlib.pyplot as plt

    df = pd.read_csv("test_data/experiment_at_2018y04m25d16H36M42S.csv")

    ts = df.values[:, 0].reshape(-1)
    mat = df.values[:, 1:]

    fig, ax = plt.subplots(nrows=1, ncols=1)
    for col_idx in range(mat.shape[1]):
        ax.plot(ts, mat[:, col_idx].reshape(-1), label='No.%s signal'%col_idx)
    ax.legend()
    ax.set_title("Offline Signals")
    plt.show()
        
