import matplotlib.pyplot as plt
import matplotlib

import numpy as np
import os
from datetime import datetime
import icecream as ic

# plt.switch_backend("Agg")
plt.ion()


class Utils(object):
    def __init__(self):
        self.path = "./TrainingData"
        if not os.path.exists(self.path):
            os.mkdir(self.path)
            print("Folder %s created!" % self.path)
        else:
            print("Folder %s already exists" % self.path)

        current_datetime = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        self.save_path = os.path.join(self.path, "EXECUTION_" + str(current_datetime))

        os.mkdir(self.save_path)

        self.header = "Episode, Epsilon, Reward, Meam Reaward"

        # creating grid for subplots
        self.fig = plt.figure()
        self.fig.set_figheight(6)
        self.fig.set_figwidth(12)

        self.ax11 = plt.subplot2grid(shape=(2, 2), loc=(0, 0), colspan=2)
        self.ax11.set_xlabel("Episodes")
        self.ax11.set_ylabel("Epsilon")
        self.ax11.set_ylim([-0.05, 1.05])
        (self.h11,) = self.ax11.plot([], [], "k", label="Epsilon")
        self.ax11.set_yticks([0, 0.25, 0.5, 0.75, 1])
        self.ax12 = self.ax11.twinx()
        self.ax12.set_ylabel("Reward")
        (self.h12,) = self.ax12.plot([], [], "r", label="Score")
        (self.h13,) = self.ax12.plot([], [], "b:", label="Meam score")
        plt.legend(
            [self.h11, self.h12, self.h13],
            [
                self.h11.get_label(),
                self.h12.get_label(),
                self.h13.get_label(),
            ],
        )

        self.curent_reward = 0
        self.curent_m_reward = 0

        self.ax2 = plt.subplot2grid(shape=(2, 2), loc=(1, 0), colspan=1)
        self.ax2.set_xlabel("Episodes")
        self.ax2.set_ylabel("Distance (m)")
        (self.h21,) = self.ax2.plot([], [], "r", label="Current")
        (self.h22,) = self.ax2.plot([], [], "b:", label="Meam")
        plt.legend(
            [self.h21, self.h22],
            [
                self.h21.get_label(),
                self.h22.get_label(),
            ],
        )

        self.ax3 = plt.subplot2grid(shape=(2, 2), loc=(1, 1), colspan=1)
        self.ax3.set_xlabel("Episodes")
        self.ax3.set_ylabel("Distance (m)")
        (self.h31,) = self.ax3.plot([], [], "r", label="Current")
        (self.h32,) = self.ax3.plot([], [], "b:", label="Meam")
        plt.legend(
            [self.h31, self.h32],
            [
                self.h31.get_label(),
                self.h32.get_label(),
            ],
        )

        plt.tight_layout()

        self.mMeamSize = 20
        self.data1 = []  # epsilon
        self.data2 = []  # reward
        self.data3 = []  # something1
        self.data4 = []  # something2

        self.meam_data2 = []  # meam reward
        self.meam_data3 = []  # meam something1
        self.meam_data4 = []  # meam something2

    def updatePlot(self, epsilom, reward, smt1, smt2):
        self.data1.append(epsilom)
        self.data2.append(reward)
        self.data3.append(smt1)
        self.data4.append(smt2)

        self.meam_data2.append(self.m_meam(self.data2))
        self.meam_data3.append(self.m_meam(self.data3))
        self.meam_data4.append(self.m_meam(self.data4))

        self.curent_reward = self.data2[-1]
        self.curent_m_reward = self.meam_data2[-1]

        x = [i + 1 for i in range(len(self.data1))]

        self.h11.set_xdata(x)
        self.h11.set_ydata(self.data1)

        self.h12.set_xdata(x)
        self.h12.set_ydata(self.data2)

        self.h13.set_xdata(x)
        self.h13.set_ydata(self.meam_data2)

        self.ax12.ignore_existing_data_limits = True
        ax12_lim = ((0, min(self.h12.get_ydata())), (len(x), max(self.h13.get_ydata())))
        self.ax12.update_datalim(ax12_lim)
        self.ax12.autoscale_view()

        self.h21.set_xdata(x)
        self.h21.set_ydata(self.data3)

        self.h22.set_xdata(x)
        self.h22.set_ydata(self.meam_data3)

        self.ax2.ignore_existing_data_limits = True
        ax2_lim = ((0, min(self.h21.get_ydata())), (len(x), max(self.h21.get_ydata())))
        self.ax2.update_datalim(ax2_lim)
        self.ax2.autoscale_view()

        self.h31.set_xdata(x)
        self.h31.set_ydata(self.data4)

        self.h32.set_xdata(x)
        self.h32.set_ydata(self.meam_data4)

        self.ax3.ignore_existing_data_limits = True
        ax3_lim = ((0, min(self.h31.get_ydata())), (len(x), max(self.h31.get_ydata())))
        self.ax3.update_datalim(ax3_lim)
        self.ax3.autoscale_view()

        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        # plt.show()

    def m_meam(self, _data):
        return np.mean(_data[max(0, len(_data) - self.mMeamSize) : (len(_data) + 1)])

    def saveLearningData(self):
        plt.savefig(self.save_path + "/Learning_Data.pdf")

        rows = [
            self.h11.get_xdata(),
            self.h11.get_ydata(),
            self.h12.get_ydata(),
            self.h13.get_ydata(),
            self.h21.get_ydata(),
            self.h22.get_ydata(),
            self.h31.get_ydata(),
            self.h32.get_ydata(),
        ]

        np.savetxt(
            os.path.join(self.save_path, "DataLearning.csv"),
            np.column_stack(rows),
            delimiter=", ",
            fmt="% s",
            comments="",
            header=self.header,
        )
