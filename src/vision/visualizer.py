#!/usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys


filename = sys.argv[1]

df = pd.read_csv(filename)

print df.describe()

def scatter(df_, n):
    color = ['r' if cl == 1 else 'b' for cl in df_["class"].values]
    ax1 = plt.subplot(421, xlabel="Hue", ylabel="Saturation")#, sharex=axes[0])
    ax1.scatter(df_["H"].values, df_["S"].values, c=color, s=50, alpha=0.05)
    # ax1.set_xlabel("hue")
    # ax2.ylabel("saturation")

    # ax2 = plt.subplot(422, xlabel="Hue", ylabel="Value")#, sharex=axes[0])
    # ax2.scatter(df_["H"].values, df_["V"].values, c=color, s=50, alpha=0.05)

    ax2 = plt.subplot(422, sharex=ax1, xlabel="Hue")
    ax2.hist(df_[df_["class"]==1]["H"].values, normed=1, bins=50, color='red')

    ax3 = plt.subplot(423, sharex=ax1, xlabel="Hue")
    ax3.hist(df_[df_["class"]==1]["H"].values, normed=0, bins=50, color='red')

    # ax3 = plt.subplot(423, xlabel="Saturation", ylabel="Value")#, sharex=axes[0])
    # ax3.scatter(df_["S"].values, df_["V"].values, c=color, s=50, alpha=0.05)

    ax4 = plt.subplot(424, sharex=ax3, sharey=ax3, xlabel="Hue")
    ax4.hist(df_[df_["class"]==0]["H"].values, normed=0, bins=50, color='blue')

    ax5 = plt.subplot(425, xlabel="Saturation")
    ax5.hist(df_[df_["class"]==1]["S"].values, normed=1, bins=50, color='red')

    ax6 = plt.subplot(426, sharex=ax5, xlabel="Value")
    ax6.hist(df_[df_["class"]==0]["S"].values, normed=0, bins=50, color='blue')


    ax7 = plt.subplot(427, xlabel="Value")
    ax7.hist(df_[df_["class"]==1]["V"].values, normed=1, bins=50, color='red')

    ax8 = plt.subplot(428, sharex=ax5, xlabel="Value")
    ax8.hist(df_[df_["class"]==0]["V"].values, normed=0, bins=50, color='blue')






scatter(df, 1)

plt.show()