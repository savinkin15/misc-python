import open3d as o3d
import numpy as np
import pandas as pd
import copy
import cv2
import csv
#This script is using open3d version 0.13.0

filename = "RGBD_sim_real"

def deproject_pixel_to_point(x,y,depth):
    px = (x-intrinsics.intrinsic_matrix[0,2]) / intrinsics.intrinsic_matrix[0,0]
    py = (y-intrinsics.intrinsic_matrix[1,2]) / intrinsics.intrinsic_matrix[1,1]
    return [depth*px,depth*py,depth]

def project_point_to_pixel(x,y,z,tf):
    #protects zero division erros
    if(z==0):
        z = 0.00000001
    cam_pts = np.array([x,y,z,1])
    cam_pts = np.linalg.inv(tf)@cam_pts
    px = (cam_pts[0]/cam_pts[2]) * intrinsics.intrinsic_matrix[0,0] + intrinsics.intrinsic_matrix[0,2]
    py = (cam_pts[1]/cam_pts[2]) * intrinsics.intrinsic_matrix[1,1] + intrinsics.intrinsic_matrix[1,2]
    return [int(px),int(py)]

def draw_bulls_eye(image,px,py):
    image = cv2.circle(image,(point[0],point[1]),7,(0,255,0),2)
    image = cv2.circle(image,(point[0],point[1]),2,(0,0,255),-1)
    return image

def draw_bulls_eye_from_3d(image,x,y,z,tf):
    pts = project_point_to_pixel(x,y,z,tf)
    image = cv2.circle(image,(pts[0],pts[1]),7,(0,255,0),2)
    image = cv2.circle(image,(pts[0],pts[1]),2,(0,0,255),-1)
    return image

points = []
points_3d = []
def mousePoint(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        depth = image_d[y,x]
        print("Coordinates of pixel: X: ",x,"Y: ",y,"Depth: ",depth)
        points.append([x,y,depth])
        point_3d = deproject_pixel_to_point(x,y,depth)
        point_3d = np.array([point_3d[0],point_3d[1],point_3d[2],1])
        # print(point_3d)
        # print(pose)
        point_3d = pose@point_3d
        # print(point_3d)
        points_3d.append(point_3d)
        # print(deproject_pixel_to_point(x,y,depth))

class CameraPose:

    def __init__(self, meta, mat):
        self.metadata = meta
        self.pose = mat

    def __str__(self):
        return 'Metadata : ' + ' '.join(map(str, self.metadata)) + '\n' + \
            "Pose : " + "\n" + np.array_str(self.pose)


def read_trajectory(filename):
    traj = []
    with open(filename, 'r') as f:
        metastr = f.readline()
        while metastr:
            metadata = list(map(int, metastr.split()))
            mat = np.zeros(shape=(4, 4))
            for i in range(4):
                matstr = f.readline()
                mat[i, :] = np.fromstring(matstr, dtype=float, sep=' \t')
            traj.append(CameraPose(metadata, mat))
            metastr = f.readline()
    return traj

camera_poses = read_trajectory(filename+"/odometry.log")
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=4.0 / 1024.0,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
rgbd_images = []
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
for i in range(len(camera_poses)):
    print("Integrate {:d}-th image into the volume.".format(i))
    color = o3d.io.read_image(filename+"/color/{:d}.jpg".format(i))
    depth = o3d.io.read_image(filename+"/depth/{:d}.png".format(i))
    print(np.asarray(color).shape,np.asarray(depth).shape)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=2.0, convert_rgb_to_intensity=False)
    rgbd_images.append(rgbd)
    data = pd.read_csv(filename+'/intrinsics.log',sep=',',header=None).values[0]
    intrinsics = o3d.camera.PinholeCameraIntrinsic(int(data[0]),int(data[1]),data[2],data[3],data[4],data[5])
    

    # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,intrinsics)
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # o3d.visualization.draw_geometries([pcd,axis])

    rot = np.array(
        [[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
    )
    #flips x and y for ros-->open3d
    #if we save points, we need to flip it back again
    tmp_pose = copy.deepcopy(camera_poses[i].pose)
    camera_poses[i].pose[0,3] = tmp_pose[2,3]
    camera_poses[i].pose[2,3] = tmp_pose[1,3]
    camera_poses[i].pose[1,3] = tmp_pose[0,3]

    volume.integrate(
    rgbd,
    intrinsics,
    np.linalg.inv(rot@camera_poses[i].pose))

id = 0
max_id = len(rgbd_images)
cv2.namedWindow('mousePoint')
cv2.setMouseCallback('mousePoint',mousePoint)
while(1):
    image = np.asarray(rgbd_images[id].color)
    image_d = np.asarray(rgbd_images[id].depth)
    pose = camera_poses[id].pose
    image_draw = cv2.cvtColor(copy.deepcopy(image),cv2.COLOR_BGR2RGB)
    for point in points_3d:
        draw_bulls_eye_from_3d(image_draw,point[0],point[1],point[2],pose)
    cv2.imshow('mousePoint',image_draw)
    #not the nicest UI. needs to press couple of times to act correctly..
    if cv2.waitKey(20) & 0xFF == 27:
        break
    if cv2.waitKey(20) == ord('a'):
        id -=1
        if(id <0):
            id = 0
        print("goto previous image")
    if cv2.waitKey(20) == ord('d'):
        id +=1
        if(id>max_id-1):
            id=max_id-1
        print("goto next image")
#if esc pressed, finish.
cv2.destroyAllWindows()
viz_meshes = []
mesh = volume.extract_triangle_mesh()
mesh.compute_vertex_normals()
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[pose[0,3], pose[1,3], pose[2,3]])
axis_point = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[points_3d[0][0],points_3d[0][1], points_3d[0][2]])
viz_meshes.append(mesh)
viz_meshes.append(axis)
file = open("test.csv",'w')
writer = csv.writer(file,lineterminator='\n')
for point in points_3d:
    axis_point = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[point[0],point[1], point[2]])
    viz_meshes.append(axis_point)
    writer.writerow(point)
file.close()
o3d.visualization.draw_geometries(viz_meshes)