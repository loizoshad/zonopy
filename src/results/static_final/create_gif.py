# Given a series of png files create a gif out of them

import os
import imageio
from pdf2image import convert_from_path
# max_N = 144
max_N = 145

def create_gif(filenames, duration):
    images = []
    for filename in filenames:
        images.append(imageio.imread(filename))
    output_file = 'static_env_brs.gif'
    imageio.mimsave(output_file, images, duration=duration)

    # images = []
    # for filename in filenames:
    #     pages = convert_from_path(filename, 200)  # convert PDF to PIL images
    #     for page in pages:
    #         images.append(page)
    # output_file = 'brs_N_{max_N}.gif'
    # imageio.mimsave(output_file, images, duration=duration)    


if __name__ == "__main__":
    script_dir = os.path.dirname(__file__)
    # TODO: Change to relative path
    results_dir = os.path.join(script_dir, '/home/loizos/Desktop/MSc/Thesis/Dev/zonopy/src/results/static_final/png_step_025')
    os.chdir(results_dir)

    n = 0
    filenames = []
    for N in range(0, max_N):
        filename = f'brs_{n}.png'
        filenames.append(filename)
        n = N + 1

    duration = 10.0
    create_gif(filenames, duration)