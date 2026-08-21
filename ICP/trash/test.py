import os
import glob
import shutil  # for copying files

src_folder = "/home/ashutosh/Documents/ICP/rgbd_dataset_freiburg1_desk/depth"
dst_folder = "/home/ashutosh/Documents/ICP/yoyo"

# make new folder if it doesn't exist
os.makedirs(dst_folder, exist_ok=True)

files = sorted(glob.glob(os.path.join(src_folder, "*.png")))

for idx, file in enumerate(files, start=0):
    new_name = os.path.join(dst_folder, f"{idx}.png")   # or f"{idx:06d}.png" for zero-padded
    shutil.copy(file, new_name)  # copy instead of rename
    print(f"Copied {file} → {new_name}")
