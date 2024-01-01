"""### Part 2: Fitting a 3D Image"""

import os
import gdown
import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import torch.nn.functional as F
import time

import math
def positional_encoding(x, num_frequencies=6, incl_input=True):

    """
    Apply positional encoding to the input.

    Args:
    x (torch.Tensor): Input tensor to be positionally encoded.
      The dimension of x is [N, D], where N is the number of input coordinates,
      and D is the dimension of the input coordinate.
    num_frequencies (optional, int): The number of frequencies used in
     the positional encoding (default: 6).
    incl_input (optional, bool): If True, concatenate the input with the
        computed positional encoding (default: True).

    Returns:
    (torch.Tensor): Positional encoding of the input tensor.
    """

    results = []
    if incl_input:
        results.append(x)

    #############################  TODO 1(a) BEGIN  ############################
    # encode input tensor and append the encoded tensor to the list of results.

    for i in range(num_frequencies):
      # sin_pos = torch.sin(2**i*math.pi*x)
      # cos_pos = torch.cos(2**i*math.pi*x)
      results.append(torch.sin(2**i*math.pi*x))
      results.append(torch.cos(2**i*math.pi*x))


    #############################  TODO 1(a) END  ##############################
    return torch.cat(results, dim=-1)

"""2.1 Complete the following function that calculates the rays that pass through all the pixels of an HxW image"""

def get_rays(height, width, intrinsics, w_R_c, w_T_c):

    """
    Compute the origin and direction of rays passing through all pixels of an image (one ray per pixel).

    Args:
    height: the height of an image.
    width: the width of an image.
    intrinsics: camera intrinsics matrix of shape (3, 3).
    w_R_c: Rotation matrix of shape (3,3) from camera to world coordinates.
    w_T_c: Translation vector of shape (3,1) that transforms

    Returns:
    ray_origins (torch.Tensor): A tensor of shape (height, width, 3) denoting the centers of
      each ray. Note that desipte that all ray share the same origin, here we ask you to return
      the ray origin for each ray as (height, width, 3).
    ray_directions (torch.Tensor): A tensor of shape (height, width, 3) denoting the
      direction of each ray.
    """

    device = intrinsics.device
    ray_directions = torch.zeros((height, width, 3), device=device)  # placeholder
    ray_origins = torch.zeros((height, width, 3), device=device)  # placeholder

    #############################  TODO 2.1 BEGIN  ##########################
    Kinv = torch.inverse(intrinsics)
    w_R_c = torch.Tensor(w_R_c).to(device)
    ray_origins[:, :] = torch.Tensor(w_T_c).to(device)
    for i in range(width):
        for j in range(height):
            ray_directions[j, i, :]  = w_R_c @ (Kinv @ torch.tensor([i, j, 1.0], device=device))
            
    #############################  TODO 2.1 END  ############################
    return ray_origins, ray_directions

"""Complete the next function to visualize how is the dataset created. You will be able to see from which point of view each image has been captured for the 3D object. What we want to achieve here, is to being able to interpolate between these given views and synthesize new realistic views of the 3D object."""

def plot_all_poses(poses):

    #############################  TODO 2.1 BEGIN  ############################
    origins = []
    directions = []
    for i in range(poses.shape[0]):
      w_R_c = poses[i][:3,:3].T
      w_T_c = poses[i][:3,-1]
      orig, dir = get_rays(height, width, intrinsics,w_R_c, w_T_c)
      origins.append(orig)
      directions.append(dir)
    origins = torch.stack(origins).cpu() #Convert list to Tensor
    directions = torch.stack(directions).cpu()

    #############################  TODO 2.1 END  ############################

    print(origins.shape)
    ax = plt.figure(figsize=(12, 8)).add_subplot(projection='3d')
    _ = ax.quiver(origins[..., 0].flatten(),
                  origins[..., 1].flatten(),
                  origins[..., 2].flatten(),
                  directions[..., 0].flatten(),
                  directions[..., 1].flatten(),
                  directions[..., 2].flatten(), length=0.12, normalize=True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('z')
    plt.show()



"""2.2 Complete the following function to implement the sampling of points along a given ray."""

def stratified_sampling(ray_origins, ray_directions, near, far, samples):

    """
    Sample 3D points on the given rays. The near and far variables indicate the bounds of sampling range.

    Args:
    ray_origins: Origin of each ray in the "bundle" as returned by the
      get_rays() function. Shape: (height, width, 3).
    ray_directions: Direction of each ray in the "bundle" as returned by the
      get_rays() function. Shape: (height, width, 3).
    near: The 'near' extent of the bounding volume.
    far:  The 'far' extent of the bounding volume.
    samples: Number of samples to be drawn along each ray.

    Returns:
    ray_points: Query 3D points along each ray. Shape: (height, width, samples, 3).
    depth_points: Sampled depth values along each ray. Shape: (height, width, samples).
    """

    #############################  TODO 2.2 BEGIN  ############################
    height, width, _ = ray_origins.shape
    depth_range = far - near
    sample_indices = torch.arange(samples, dtype=torch.float32) - 1
    sample_ratios = sample_indices / samples
    depth_values = depth_range * sample_ratios + near

    # find ray origins, ray directions
    ray_origins = ray_origins.reshape(height, width, 1, 3)
    ray_directions = ray_directions.reshape(height, width, 1, 3)

    # find depth_points, ray points
    depth_points = torch.broadcast_to(depth_values, (height, width, samples))
    ray_points = ray_origins + depth_points.reshape((height, width, samples, 1)) * ray_directions
    
    #############################  TODO 2.2 END  ############################
    return ray_points, depth_points

"""2.3 Define the network architecture of NeRF along with a function that divided data into chunks to avoid memory leaks during training."""

class nerf_model(nn.Module):

    """
    Define a NeRF model comprising eight fully connected layers and following the
    architecture described in the NeRF paper.
    """

    def __init__(self, filter_size=256, num_x_frequencies=6, num_d_frequencies=3):
        super().__init__()

        #############################  TODO 2.3 BEGIN  ############################
        # for autograder compliance, please follow the given naming for your layers
        self.input_dim_x = 2*(3*num_x_frequencies) + 3
        self.input_dim_d = 2*(3*num_d_frequencies) + 3
        self.layers = nn.ModuleDict({
            'layer_1': nn.Linear(in_features= self.input_dim_x, out_features=filter_size),
            'layer_2': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_3': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_4': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_5': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_6': nn.Linear(in_features= filter_size + 2*3*num_x_frequencies + 3, out_features=filter_size),
            'layer_7': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_8': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_s': nn.Linear(in_features= filter_size, out_features=1),
            'layer_9': nn.Linear(in_features= filter_size, out_features=filter_size),
            'layer_10': nn.Linear(in_features= filter_size + self.input_dim_d, out_features=128),
            'layer_11': nn.Linear(in_features= 128, out_features=3)
        })

        #############################  TODO 2.3 END  ############################


    def forward(self, x, d):
        #############################  TODO 2.3 BEGIN  ############################
        # example of forward through a layer: y = self.layers['layer_1'](x)

        l1_out = torch.relu(self.layers['layer_1'](x))
        l2_out = torch.relu(self.layers['layer_2'](l1_out))
        l3_out = torch.relu(self.layers['layer_3'](l2_out))
        l4_out = torch.relu(self.layers['layer_4'](l3_out))
        l5_out = torch.relu(self.layers['layer_5'](l4_out))

        l6_out = torch.relu(self.layers['layer_6'](torch.cat([l5_out, x], dim=-1)))
        l7_out = torch.relu(self.layers['layer_7'](l6_out))
        l8_out = torch.relu(self.layers['layer_8'](l7_out))
        sigma = self.layers['layer_s'](l8_out)
        l9_out = self.layers['layer_9'](l8_out)


        l10_out = torch.relu(self.layers['layer_10'](torch.cat([l9_out, d], dim= -1)))
        rgb = torch.sigmoid(self.layers['layer_11'](l10_out))
        #############################  TODO 2.3 END  ############################
        return rgb, sigma

def get_batches(ray_points, ray_directions, num_x_frequencies, num_d_frequencies):

    def get_chunks(inputs, chunksize = 2**15):
        """
        This fuction gets an array/list as input and returns a list of chunks of the initial array/list
        """
        return [inputs[i:i + chunksize] for i in range(0, inputs.shape[0], chunksize)]

    """
    This function returns chunks of the ray points and directions to avoid memory errors with the
    neural network. It also applies positional encoding to the input points and directions before
    dividing them into chunks, as well as normalizing and populating the directions.
    """
    #############################  TODO 2.3 BEGIN  ############################
    ray_directions_norm = ray_directions / torch.norm(ray_directions, dim=-1, keepdim=True)

    # repeat directions for every point
    ray_directions_pop = ray_directions_norm.unsqueeze(-2).repeat(1, 1, ray_points.shape[2], 1)

    # flatten vectors
    ray_points_flat = ray_points.view(-1, 3) #(H * W * N of samples, 3)
    ray_directions_flat = ray_directions_pop.view(-1, 3) #(H * W * N of samples, 3)

    # Apply positional encoding:
    en_ray_directions_flat = positional_encoding(ray_directions_flat, num_d_frequencies)
    en_ray_points_flat = positional_encoding(ray_points_flat, num_x_frequencies)

    # Call get_chunks:
    ray_points_batches = get_chunks(en_ray_points_flat)
    ray_directions_batches = get_chunks(en_ray_directions_flat)
    #############################  TODO 2.3 END  ############################

    return ray_points_batches, ray_directions_batches

"""2.4 Compute the compositing weights of samples on camera ray and then complete the volumetric rendering procedure to reconstruct a whole RGB image from the sampled points and the outputs of the neural network."""

def volumetric_rendering(rgb, s, depth_points):

    """
    Differentiably renders a radiance field, given the origin of each ray in the
    "bundle", and the sampled depth values along them.

    Args:
    rgb: RGB color at each query location (X, Y, Z). Shape: (height, width, samples, 3).
    sigma: Volume density at each query location (X, Y, Z). Shape: (height, width, samples).
    depth_points: Sampled depth values along each ray. Shape: (height, width, samples).

    Returns:
    rec_image: The reconstructed image after applying the volumetric rendering to every pixel.
    Shape: (height, width, 3)
    """

    device = "cpu"
    #############################  TODO 2.4 BEGIN  ############################
    delta_i = 1e9*torch.ones_like(depth_points).to(device)
    delta_i[:, :, :-1] = torch.diff(depth_points, dim=-1)
    sigma_i = -F.relu(s)*delta_i
    T_i = torch.cumprod(torch.exp(sigma_i), dim=-1)
    T_i = torch.roll(T_i, 1, dims=-1)
    AdjSigmaDel_i = 1 - torch.exp(sigma_i)
    rec_image = ((T_i * AdjSigmaDel_i)[..., None] * rgb).sum(dim=-2)
    #############################  TODO 2.4 END  ############################

    return rec_image

"""To test and visualize your implementation, independently of the previous and next steps of the
NeRF pipeline, you can load the sanity_volumentric.pt file, run your implementation of the volumetric function and expect to see the figure provided in the handout.

"""

"""2.5 Combine everything together. Given the pose position of a camera, compute the camera rays and sample the 3D points along these rays. Divide those points into batches and feed them to the neural network. Concatenate them and use them for the volumetric rendering to reconstructed the final image."""

def one_forward_pass(height, width, intrinsics, pose, near, far, samples, model, num_x_frequencies, num_d_frequencies):

    #############################  TODO 2.5 BEGIN  ############################

    #compute all the rays from the image
    ray_origins, ray_directions = get_rays(height,width, intrinsics, pose[:3, :3].T , pose[:3, -1])

    #sample the points from the rays
    ray_points, depth_points = stratified_sampling(ray_origins, ray_directions, near, far, samples)
    # pdb.set_trace()
    #divide data into batches to avoid memory errors
    ray_points_batches, ray_directions_batches = get_batches(ray_points, ray_directions, num_x_frequencies, num_d_frequencies)

    #forward pass the batches and concatenate the outputs at the end
    rgb_list = []
    s_list = []
    device = "cpu"
    for i in range(len(ray_points_batches)):
      rgb_i, s_i = model(ray_points_batches[i].to(device), ray_directions_batches[i].to(device))
      # print(f"RGB ={rgb_i.shape}, S = {s_i.shape}")
      rgb_list.append(rgb_i)
      s_list.append(s_i)

    # pdb.set_trace()
    rgb = torch.cat(rgb_list, dim = 0).reshape(height, width, samples, 3)
    s = torch.vstack(s_list).reshape(height, width, samples)
    # Apply volumetric rendering to obtain the reconstructed image
    rec_image = volumetric_rendering(rgb, s, depth_points)

    #############################  TODO 2.5 END  ############################

    return rec_image

"""If you manage to pass the autograder for all the previous functions, then it is time to train a NeRF! We provide the hyperparameters for you, we initialize the NeRF model and its weights, and we define a couple lists that will be needed to store results."""


