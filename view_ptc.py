import OpenGL.GL as gl
import pangolin
import numpy as np
from time import sleep


def main():
    h, w = 480, 752 # 640, 480
    pangolin.CreateWindowAndBind('Main', 640, 480)
    gl.glEnable(gl.GL_DEPTH_TEST)

    # Define Projection and initial ModelView matrix
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 200),
        pangolin.ModelViewLookAt(0, 0, 0, 0, 0, 1, pangolin.AxisDirection.AxisNegY))
    handler = pangolin.Handler3D(scam)

    # Create Interactive View in window
    dcam = pangolin.CreateDisplay()
    # dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0/480.0)
    dcam.SetHandler(handler)

    T_wc = scam.GetModelViewMatrix()

    X = np.fromfile('landmarks_R3.lm', sep=' ')
    X = np.reshape(X, (-1, 4))
    T = np.fromfile('poses_SE3.T', sep=' ')
    T = np.reshape(T, (-1, 4, 4))
    T_opt = np.fromfile('opt_poses_SE3.T', sep=' ')
    T_opt = np.reshape(T_opt, (-1, 4, 4))
    T_true = np.fromfile('true_poses_SE3.T', sep=' ')
    T_true = np.reshape(T_true, (-1, 4, 4))

    T_opt_norm = np.copy(T_opt)
    norm_pose_true = np.linalg.inv(T_true[0]) @ T_true[1]
    norm_pose_opt = np.linalg.inv(T_opt[0]) @ T_opt[1]
    norm_const = np.linalg.norm(norm_pose_true[:3, 3]) / np.linalg.norm(norm_pose_opt[:3, 3])
    T_opt_norm[:, :3, 3] = T_opt_norm[:, :3, 3] * norm_const

    displacement = np.linalg.inv(T_opt_norm[0]) @ T[0]
    displacement[:3, :3] = 0.0
    displacement[3, 3] = 0.0
    T_opt_norm = np.array([pose + displacement for pose in T_opt_norm])

    i = 0
    #sleep(5)
    #pangolin.DisplayBase().RecordOnRender("ffmpeg:[fps=10,bps=8388608,unique_filename]//kitti_00.avi")
    while not pangolin.ShouldQuit():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        dcam.Activate(scam)

        #T_wc = scam.GetModelViewMatrix()

        # T_i = T[i]
        # T_mv = T[0]
        # scam.SetModelViewMatrix(pangolin.OpenGlMatrix(T_mv))
        dcam.Render()

        # Draw Point Cloud Nx3
        gl.glPointSize(3)
        gl.glColor3f(1.0, 0.0, 0.0)
        pangolin.DrawPoints(X[:, :3])

        # Draw camera
        gl.glLineWidth(2)
        gl.glColor3f(0.0, 1.0, 0.0)
        for pose in T:
            pangolin.DrawCamera(pose, 0.5/2, 0.75/2, 0.8/2)
        pangolin.DrawLine(T[:, :3, 3])

        # gl.glColor3f(1.0, 0.0, 0.0)
        # for pose in T_opt:
        #     pangolin.DrawCamera(pose, 0.5/2, 0.75/2, 0.8/2)
        # pangolin.DrawLine(T_opt[:i + 1, :3, 3])

        gl.glColor3f(0.0, 0.0, 1.0)
        for pose in T_true:
            pangolin.DrawCamera(pose, 0.5/2, 0.75/2, 0.8/2)
        pangolin.DrawLine(T_true[:, :3, 3])

        gl.glColor3f(1.0, 0.0, 1.0)
        for pose in T_opt_norm:
            pangolin.DrawCamera(pose, 0.5/2, 0.75/2, 0.8/2)
        pangolin.DrawLine(T_opt_norm[:, :3, 3])

        i = min(i + 1, len(T) - 1)

        pangolin.FinishFrame()
        #sleep(0.1)

if __name__ == '__main__':
    main()
