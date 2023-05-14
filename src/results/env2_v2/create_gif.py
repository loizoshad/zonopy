# Given a list of pdf files, create a gif of the pdfs
# All the pdf files are in this directory and they are named as 'brs_N_x.pdf', where x = 0, 1, 2, ..., 10

import os
import imageio
from pdf2image import convert_from_path

def create_gif(filenames, duration):
    images = []
    for filename in filenames:
        pages = convert_from_path(filename, 200)  # convert PDF to PIL images
        for page in pages:
            images.append(page)
    output_file = 'brs_N.gif'
    imageio.mimsave(output_file, images, duration=duration)

if __name__ == "__main__":
    script_dir = os.path.dirname(__file__)
    # TODO: Change to relative path
    results_dir = os.path.join(script_dir, '/home/loizos/Desktop/MSc/Thesis/Dev/zonopy/src/results/env2_v2')
    os.chdir(results_dir)

    filenames = []
    for N in range(0, 74):
        filename = f'brs_N_{N}.pdf'
        filenames.append(filename)

    duration = 2.5
    create_gif(filenames, duration)