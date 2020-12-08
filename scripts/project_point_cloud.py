import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import time

def project_pointcloud(o3dpointcloud, im_size, T_cam, max_range):
	points = np.asarray(o3dpointcloud.points).T
	K = np.array([
		[im_size[0] / 2, -min(im_size), 0           , 0],
		[im_size[1] / 2, 0            , min(im_size), 0],
		[1             , 0            , 0           , 0]
	])

	points_homo = np.concatenate(
		[points, np.ones((1, points.shape[1]))], axis=0
	)
	points_cam_homo = np.dot(T_cam, points_homo)
	pixels_homo = np.dot(K, points_cam_homo)
	pixels = pixels_homo[0:2, :] / pixels_homo[2, :]

	image_flat = (np.ones(im_size) * np.inf).flatten()
	i = im_size[0] - np.round(pixels[1, :]).astype(np.int32)
	j = np.round(pixels[0, :]).astype(np.int32)
	valid = (i >= 0) & (i < im_size[0]) & (j >= 0) & (j < im_size[1])
	index_flat = i[valid] * im_size[1] + j[valid]
	depth = points_cam_homo[0, valid]
	infront = depth > 0
	np.minimum.at(image_flat, index_flat[infront], depth[infront])
	#from IPython import embed; embed()
	image_flat[image_flat == np.inf] = max_range
	image_flat = image_flat/max_range * 255

	return image_flat.reshape(im_size)

if __name__=="__main__":
	filename = "bunny.pcd"
	pcd = o3d.io.read_point_cloud(filename)

	T_cam = np.array([
		[1, 0, 0, .3],
		[0, 1, 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	])
	im_size = (360, 360)
	image = project_pointcloud(pcd, im_size, T_cam, 0.4)

	plt.imshow(image, vmin=np.min(image[image!=-1])-.01, cmap='gray')
	plt.colorbar()
	plt.show()