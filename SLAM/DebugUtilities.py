import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import plotly
import plotly.graph_objs as go


def check_epipolar(img1, pts1, img2, pts2):
    def drawlines(img1, img2, lines, pts1, pts2):
        ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
        r, c = img1.shape
        img1 = cv.cvtColor(img1, cv.COLOR_GRAY2BGR)
        img2 = cv.cvtColor(img2, cv.COLOR_GRAY2BGR)
        for r, pt1, pt2 in zip(lines, pts1, pts2):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            x0, y0 = map(int, [0, -r[2] / r[1]])
            x1, y1 = map(int, [c, -(r[2] + r[0] * c) / r[1]])
            img1 = cv.line(img1, (x0, y0), (x1, y1), color, 1)
            img1 = cv.circle(img1, (int(pt1[0]), int(pt1[1])), 5, color, -1)
            img2 = cv.circle(img2, (int(pt2[0]), int(pt2[1])), 5, color, -1)
        return img1, img2

    F, mask = cv.findFundamentalMat(pts1, pts2, cv.RANSAC)

    lines1 = cv.computeCorrespondEpilines(pts2.reshape(-1, 1, 2), 2, F)
    lines1 = lines1.reshape(-1, 3)
    img5, img6 = drawlines(img1, img2, lines1, pts1, pts2)
    # Find epilines corresponding to points in left image (first image) and
    # drawing its lines on right image
    lines2 = cv.computeCorrespondEpilines(pts1.reshape(-1, 1, 2), 1, F)
    lines2 = lines2.reshape(-1, 3)
    img3, img4 = drawlines(img2, img1, lines2, pts2, pts1)
    plt.subplot(121), plt.imshow(img5)
    plt.subplot(122), plt.imshow(img3)
    plt.show()


def draw_3d(xpoints, ypoints, zpoints):
    # Configure the trace.
    trace = go.Scatter3d(
        x=xpoints,  # <-- Put your data instead
        y=ypoints,  # <-- Put your data instead
        z=zpoints,  # <-- Put your data instead
        mode='markers',
        marker=dict(
            size=2,
            opacity=0.8,
            colorscale='Viridis'
        ),
        line=dict(
            color='darkblue',
            width=2
        )
    )

    # Configure the layout.
    layout = go.Layout(
        margin={'l': 0, 'r': 0, 'b': 0, 't': 0},
        scene=dict(
            camera=dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=0.2, y=-2.5, z=0)
            )
        ),
    )

    data = [trace]

    plot_figure = go.Figure(data=data, layout=layout)

    # Render the plot.
    plot_figure.show()


def display_plot(points):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    points = np.array(points)
    zdata = points[:, 2]
    xdata = points[:, 0]
    ydata = points[:, 1]
    ax.scatter3D(xdata, ydata, zdata)
    plt.show()


def draw_points(points, img, name):
    image = img
    for point in points:
        center = (int(point[0]), int(point[1]))
        image = cv.circle(image, center, radius=1, color=(0, 255, 255), thickness=3)

    cv.imwrite(name, image)
