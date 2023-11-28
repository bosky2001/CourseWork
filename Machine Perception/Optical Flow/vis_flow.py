import numpy as np
import matplotlib.pyplot as plt

def plot_flow(image, flow_image, confidence, threshmin=10):
    """
    Plot a flow field of one frame of the data.
    
    Inputs:
        - image: grayscale image - shape: (H, W)
        - flow_image: optical flow - shape: (H, W, 2)
        - confidence: confidence of the flow estimates - shape: (H, W)
        - threshmin: threshold for confidence (optional) - scalar
    """
    
    ### STUDENT CODE START ###
    
    # Useful function: np.meshgrid()
    # Hint: Use plt.imshow(<your image>, cmap='gray') to display the image in grayscale
    # Hint: Use plt.quiver(..., color='red') to plot the flow field on top of the image in a visible manner
    
    conf_t = np.where(confidence > threshmin)
    # print(threshmin)
    # print(confidence_threshold)


    flow = flow_image[conf_t]

    plt.imshow(image, cmap = 'gray')
    plt.quiver( conf_t[1], conf_t[0], (flow[:,0] * 10).astype(int), (flow[:,1] * 10).astype(int),
               angles='xy', scale_units='xy', scale=1., color='red', width=0.001)
    ### STUDENT CODE END ###

    # this function has no return value
    return





    

