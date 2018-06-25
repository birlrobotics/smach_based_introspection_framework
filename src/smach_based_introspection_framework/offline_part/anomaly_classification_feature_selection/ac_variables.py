import numpy as np
import ipdb

window_size_config_args = []
for i in np.arange(0.0, 5, 1):
    for j in np.arange(0.0, 5, 1):
        if i == 0.0 and j == 0.0:
            continue
        window_size_config_args.append([i, j])
