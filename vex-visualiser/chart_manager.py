from visualiser import PlotOdometry

class ChartManager:

    def __init__(self):
        self.plot_odometry = PlotOdometry()
        return

    def parse(self, gui_application, raw_data_string):
        try:
            if "Odom(" in raw_data_string: # Odom(x, y, theta)
                data = [float(i) for i in raw_data_string.split("Odom(")[1].split(")")[0].split(",")]; print(data);
                self.plot_odometry.plot(*data)
                return

            gui_application.console.write(raw_data_string)
            return
        except TypeError:
            return