#!/usr/bin/env python3
import numpy as np
from skimage import io, morphology, filters
import matplotlib.pyplot as plt

# --------- CONFIG ----------
INPUT  = "narrow_corridor.pgm"
OUTPUT = "narrow_corridor_clean.pgm"

# How aggressive to be
MIN_FREE_BLOB_PIXELS   = 2000   # keep only main corridor
WALL_THICKEN_PIXELS    = 1      # grow black walls by this many px
CORRIDOR_SHRINK_PIXELS = 1      # shrink white band a bit (clean edges)
# ---------------------------

print(f"Loading {INPUT} ...")
img = io.imread(INPUT)

# 0 = occupied, 254 = free, 205 = unknown (typical ROS),
# but we'll detect threshold automatically
thresh = filters.threshold_otsu(img)
free = img > thresh          # True = free (white band)

# 1) Keep only the **largest** free-space blob (the corridor ring)
print("  Removing small free-space speckles...")
free_labels, num = morphology.label(free, return_num=True, connectivity=2)
sizes = np.bincount(free_labels.ravel())
# label 0 is background
largest_label = np.argmax(sizes[1:]) + 1
corridor = free_labels == largest_label

# 2) Close small gaps in the corridor walls
print("  Closing small gaps in walls...")
corridor = morphology.binary_closing(corridor, morphology.square(3))

# 3) Optionally shrink corridor a bit to smooth jaggies
if CORRIDOR_SHRINK_PIXELS > 0:
    corridor = morphology.binary_erosion(
        corridor,
        morphology.square(CORRIDOR_SHRINK_PIXELS)
    )

# 4) Thicken walls a bit (grow obstacle space)
walls = ~corridor
if WALL_THICKEN_PIXELS > 0:
    walls = morphology.binary_dilation(
        walls,
        morphology.square(WALL_THICKEN_PIXELS)
    )

# 5) Build final clean image: 255 = free, 0 = wall
clean = np.where(~walls, 255, 0).astype(np.uint8)

io.imsave(OUTPUT, clean)
print(f"Saved cleaned map to {OUTPUT}")

# Optional debug plot
fig, axes = plt.subplots(1, 2, figsize=(10, 5))
axes[0].set_title("Original")
axes[0].imshow(img, cmap="gray")
axes[0].axis("off")

axes[1].set_title("Cleaned")
axes[1].imshow(clean, cmap="gray")
axes[1].axis("off")

plt.tight_layout()
plt.savefig("map_cleaning_comparison.png", dpi=150)
print("Saved comparison to map_cleaning_comparison.png")
plt.show()

