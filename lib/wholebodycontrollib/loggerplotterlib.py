import numpy as np
import pickle
import matplotlib.pyplot as plt
import os

class LoggerPlotter():

    def __init__(self):
        self.data = {}

    def add_data_variables(self, names : list):
        for name in names:
            self.add_data_variable(name)

    def add_data_variable(self, name : str):
        self.data[name] = []

    def append_data(self, data : dict):
        for name, data_ in data.items():
            self.append_single_data(name, data_)

    def append_single_data(self, name, data):
        self.data[name].append(data)

    def save_data_to_file(self, file_name : str):
        directory = os.path.dirname(file_name) 
        if directory:
            os.makedirs(directory, exist_ok=True) 

        file = open(file_name, "wb")
        pickle.dump(self.data, file)
        file.close()

    def load_data_from_file(self, file_name : str):
        file = open(file_name, 'rb')
        self.data = pickle.load(file)

    def plot_data(self, x_data_name : str, y_data_names : list,title : str,  y_data_labels : list=[], show_plot : bool=True, single_plot : bool=False, selected_variables : list=[], sub_titles : list=[], x_lim : list=[], y_lim : list=[], x_label=[], y_label=[], fill_x_upper_treshold = None, fill_x_upper_color = 'blue'):

        # Check all the variables have the same shape
        n_data_points = len(self.data[x_data_name])
        first_y_data_sample = self.data[y_data_names[0]][0]

        if not np.isscalar(first_y_data_sample):
            n_variables = len(first_y_data_sample)
        else:
            n_variables = 1

        for y_name in y_data_names:
            if len(self.data[y_name]) != n_data_points:
                return None
            
        if not y_data_labels:
            y_data_labels = y_data_names
            
        # Prepare legend labels
        legend_labels = [label.replace('_', ' ') for label in y_data_labels]

        # Prepare data
        x = np.stack(self.data[x_data_name])
        ys = [np.stack(self.data[y_data_name]) for y_data_name in y_data_names]

        # Check the selected variables, if it is a single list, apply it to all the ys
        if not selected_variables:
            selected_variables = []
            for y_idx, y in enumerate(ys):
                selected_variables.append([i for i in range(0, n_variables)])
        if not isinstance(selected_variables[0], list):
            selected_variables_bk = selected_variables
            selected_variables = []
            for y_idx, y in enumerate(ys):
                selected_variables.append(selected_variables_bk)

        # Create figure
        if single_plot:
            n_subplotsplots = 1
        else:
            n_subplotsplots = len(selected_variables[0])
        fig, axes = plt.subplots(n_subplotsplots)
        fig.suptitle(title, fontsize=15, usetex=False)

        if n_subplotsplots > 1:
            for ax_idx, ax in enumerate(axes):
                for y_idx, y in enumerate(ys):
                    if not isinstance(selected_variables[y_idx][ax_idx], tuple):
                        ax.plot(x, y[:, selected_variables[y_idx][ax_idx]], label=legend_labels[y_idx], linewidth=2)
                    else:
                        ax.plot(x, y[:, selected_variables[y_idx][ax_idx][0], selected_variables[y_idx][ax_idx][1]], label=legend_labels[y_idx], linewidth=2)

                if sub_titles:
                    ax.set_title(sub_titles[ax_idx], fontsize=15)
                
                # Add legend only for the last subplot
                if ax_idx == len(axes) - 1:
                   ax.legend(bbox_to_anchor=(1.0, -0.12), fontsize=12, title='', title_fontsize='12', fancybox=True, shadow=True, ncol=len(ys))

                ax.grid(True)

                if y_lim:
                    ax.set_ylim(y_lim)
                if x_lim:
                    ax.set_xlim(x_lim)
                if x_label:
                    ax.set_xlabel(x_label, fontsize=12, labelpad=10, usetex=False)
                if y_label:
                    ax.set_ylabel(y_label, fontsize=12, labelpad=10, usetex=False)
                if fill_x_upper_treshold:
                    ax.axvspan(xmin=0, xmax=fill_x_upper_treshold, color=fill_x_upper_color, alpha=0.3)

        else:
            for idx in range(0, len(selected_variables[0])):
                for y_idx, y in enumerate(ys):
                    # In this case the subtitle is appended to the label
                    if sub_titles:
                        if isinstance(sub_titles[0], list):
                            legend_label = legend_labels[y_idx] + " - " + sub_titles[y_idx][idx]
                        else:
                            legend_label = legend_labels[y_idx] + " - " + sub_titles[idx]
                    else:
                        legend_label = legend_labels[y_idx]

                    if len(y.shape) == 1:
                        axes.plot(x, y, label=legend_label, linewidth=2)
                    elif not isinstance(selected_variables[y_idx][idx], tuple):
                        axes.plot(x, y[:, selected_variables[y_idx][idx]], label=legend_label, linewidth=2)
                    else:
                        axes.plot(x, y[:, selected_variables[y_idx][idx][0], selected_variables[y_idx][idx][1]], label=legend_label, linewidth=2)
                        
            axes.legend(bbox_to_anchor=(1.0, -0.12), fontsize=12, title='', title_fontsize='12', fancybox=True, shadow=True, ncol=len(selected_variables[0]))
            axes.grid(True)

            if y_lim:
                axes.set_ylim(y_lim)
            if x_lim:
                axes.set_xlim(x_lim)
            if x_label:
                axes.set_xlabel(x_label, fontsize=12, labelpad=10, usetex=False)
            if y_label:
                axes.set_ylabel(y_label, fontsize=12, labelpad=10, usetex=False)
            if fill_x_upper_treshold:
                axes.axvspan(xmin=0, xmax=fill_x_upper_treshold, color=fill_x_upper_color, alpha=0.5, label="test")

        plt.subplots_adjust(top=0.9, hspace=0.5, bottom=0.2)

        if show_plot:
            plt.show()

        return fig
    
    def compute_new_variable(self, input_data_names, output_data_name, operator, *args):
        self.add_data_variable(output_data_name)

        for idx in range(len(self.data[input_data_names[0]])):
            # Collect the corresponding values from input_data_names
            input_values = [self.data[name][idx] for name in input_data_names]
            # Apply the operator to the collected values and any additional arguments
            result = operator(*input_values, *args)
            # Append the result to the new variable
            self.append_single_data(output_data_name, result)
