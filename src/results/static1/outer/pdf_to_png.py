import os
import imageio
from pdf2image import convert_from_path

dir_path = '/home/loizos/Desktop/MSc/Thesis/Dev/zonopy/src/results/env3/outer'

n_images = 113


# Loop through all iamges with name brs_N_{i}.pdf at the path dir_path
i = 0
j = 0
while True:
    if i > n_images:
        break
    # Create the path to the pdf file
    pdf_path = os.path.join(dir_path, f'brs_N_{i}.pdf')
    # Convert the pdf to png
    images = convert_from_path(pdf_path)
    # Save the png in the path dir_path + 'pngs' with name brs_N_{i}.png
    images[0].save(os.path.join(dir_path, 'pngs', f'brs_N_{j}.png'))    
    i = i + 3
    j = j + 1


