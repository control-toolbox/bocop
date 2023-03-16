"""
    Module for widgets helping in the diagnostic of the results.
"""

# Prerequisite: the jupyter notebook MUST include the magic command %matplotlib widget, in order
# to use the interactive backend for matplotlib in Jupyter.

# Low level diagnostic
# - low_diagnose: display an indicator function.
import matplotlib.pyplot as plt
import numpy
from IPython.display import display
from ipywidgets import FloatSlider
from ipywidgets.widgets import VBox, Output

selected_idx = -1
has_moved = False
points = []


def low_diagnose(solution):
    """
    Setup an interactive widget allowing to display as an indicator function the parts:
    * exceeding the threshold (seen as 0);
    * lower than the threshold (seen as 1).

    Parameters
    ----------

    solution: bocop solution
    """

    def indicator(values, threshold):
        """
        Returns the indicator function.

        Parameters
        ----------
        values Outputs of a function.
        threshold

        Returns
        An array where return[i] is equal to:
          * 0 if values[i] < threshold
          * 1 otherwise
        -------

        """
        return numpy.where( numpy.abs( values ) < threshold, 1, 0)

    fig, (plot_control, plot_phi, plot_indicator) = plt.subplots(3, 1)
    plot_indicator.set_ylim(-0.05,1.05) # 0.05 = margin

    fig.suptitle("Control and threshold-controlled indicator functions")
    fig.tight_layout(pad=3.0)

    control = solution.control[0]
    time_stages = solution.time_stages

    # Display the original function
    plot_control.plot(time_stages, control)
    plot_control.set_title("Control function")

    u = control / numpy.max( control ) # Normalisation of control
    phi = u * ( 1 - u )
    plot_phi.plot( time_stages, phi )
    plot_phi.set_title( "phi = u * ( 1 - u )")

    # Display the indicator with threshold
    plot_indicator.plot(time_stages, indicator( phi, numpy.amax(control)))
    plot_indicator.set_title("Indicator function with threshold = {:.2f}".format(float(numpy.amax( phi ))))

    def update_lines(new_value):
        phi_s = new_value.new
        plot_indicator.cla()
        plot_indicator.set_autoscale_on(False)
        plot_indicator.set_ylim( -0.05, 1.05 )
        plot_indicator.set_xlim( plot_phi.get_xlim())
        plot_indicator.plot(time_stages, indicator( phi, phi_s ))
        plot_indicator.set_title("Indicator function with threshold = {:.2f}".format(phi_s))

        # option 1: a slider change will keep red lines
        for idx, (c, p, i) in enumerate(points):
            i.remove() # maybe deleted by cla()
            line_indicator = plot_indicator.axvline(c.get_xdata()[0], color='red')
            points[idx] = (c, p, line_indicator)

        # option 2: a slider move will delete red lines
        #clear points
        #for (c, p, i) in points:
        #    c.remove()
        #    p.remove()
        #    i.remove()
        #points.clear()
        #out.clear_output()

    slider = FloatSlider(
        orientation='horizontal',
        description='Threshold:',
        min=0,
        max=float(numpy.amax(phi)),
        value=float(numpy.amax(phi)),
        step=0.01,
        continuous_update=False,
        readout_format=".2f"
    )
    slider.layout.margin = '0px 30% 0px 30%'
    slider.layout.width = '40%'
    slider.observe(update_lines, names='value')

    out = Output(layout={'border': '1px solid black'})

    # click anywhere -> add point
    # click on point, move, release -> move point
    # click on point, release without moving -> remove point
    def near_point(x, max_distance=0.05):
        global points
        min_dist = max_distance
        idx = -1
        if len(points) > 0:
            for i, (_, p, _) in enumerate(points):
                distance = abs(p.get_xdata()[0] - x)
                if distance < min_dist:
                    min_dist = distance
                    idx = i
        return idx, min_dist

    def onpress(event):
        global selected_idx
        global has_moved
        global points

        if event.inaxes != plot_phi:
            return

        has_moved = False
        selected_idx, distance = near_point(event.xdata)
        if selected_idx == -1:
            line_control = plot_control.axvline(event.xdata, color='red')
            line_phi = plot_phi.axvline(event.xdata, color='red')
            line_indicator = plot_indicator.axvline(event.xdata, color='red')
            points.append((line_control, line_phi, line_indicator))
        out_points()

    def onmotion(event):
        global selected_idx
        global has_moved
        if selected_idx != -1:
            (l_control, l_phi, l_indicator) = points[selected_idx]
            l_control.set_xdata([event.xdata])
            l_phi.set_xdata([event.xdata])
            l_indicator.set_xdata([event.xdata])
            has_moved = True

    def onrelease(event):
        global selected_idx
        global has_moved
        global points

        if selected_idx != -1:
            release_idx, _ = near_point(event.xdata)
            (c, p, i) = points[selected_idx]

            if not has_moved and release_idx == selected_idx:
                c.remove()
                p.remove()
                i.remove()
                del points[selected_idx]
            else:
                c.set_xdata([event.xdata])
                p.set_xdata([event.xdata])
                i.set_xdata([event.xdata])

        selected_idx = -1
        out_points()

    def out_points():
        global points
        time_steps = solution.time_steps
        control = solution.control
        state = solution.state
        costate = solution.costate

        def findIdx(x):
            idx_time, idx_stage = 0, 0

            if x < time_steps[0]:
                idx_time = 0
            elif x > time_steps[-1]:
                idx_time = len(time_steps) -1
            else:
                while x > time_steps[idx_time]:
                    idx_time = idx_time + 1

            if x < time_stages[0]:
                idx_stage = 0
            elif x > time_steps[-1]:
                idx_stage = len(time_stages) -1
            else:
                while x > time_stages[idx_stage]:
                    idx_stage = idx_stage + 1

            return (idx_time, idx_stage)

        out.clear_output()
        with out:
            print("x, control, state, costate")
            for c, p, i in points:
                (idx_time, idx_stage) = findIdx(c.get_xdata()[0])
                # x, control, state, costate
                cc = [c[idx_stage] for c in control]
                ss = [s[idx_time] for s in state]
                co = [c[idx_stage] for c in costate]
                print([c.get_xdata()[0], cc, ss, co])

    plot_control.set_autoscale_on(False)
    plot_phi.set_autoscale_on(False)
    plot_indicator.set_autoscale_on(False)
    fig.canvas.mpl_connect('button_press_event', onpress)
    fig.canvas.mpl_connect('button_release_event', onrelease)
    fig.canvas.mpl_connect('motion_notify_event', onmotion)
    display( VBox( [ slider, out] ) )
