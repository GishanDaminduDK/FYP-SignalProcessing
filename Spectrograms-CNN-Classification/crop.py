# -*- coding: utf-8 -*-
"""
Created on Mon Oct 21 19:38:21 2024

@author: User
"""

import os
from PIL import Image

# Function to crop and save the image
def crop_image(input_path, output_path, start_coords, end_coords):
    try:
        # Open an image file
        with Image.open(input_path) as img:
            # Crop the image
            cropped_img = img.crop((*start_coords, *end_coords))
            # Save the cropped image
            cropped_img.save(output_path)
            print(f'Cropped image saved at: {output_path}')
    except Exception as e:
        print(f'Error cropping image {input_path}: {e}')

def crop_images_in_folder(input_folder, output_folder, start_coords, end_coords):
    # Create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    # Iterate over all files in the input folder
    for filename in os.listdir(input_folder):
        if filename.endswith('.png'):
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)
            # Crop and save each image
            crop_image(input_path, output_path, start_coords, end_coords)

# Example usage
input_folder = r"D:\DroneSwarmsImages\Wavelet_Plots\mavic3_mavic2_original"  # Replace with your folder path
output_folder = r"D:\DroneSwarmsImages\Wavelet_Plots\mavic3_mavic2"  # Replace with your output folder path
start_coords = (114, 50)  # Replace with the top-left coordinates (x1, y1)
end_coords = (696,540)    # Replace with the bottom-right coordinates (x2, y2)

# Call the function to crop images
crop_images_in_folder(input_folder, output_folder, start_coords, end_coords)
