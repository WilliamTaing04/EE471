# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Vision-Based Robotic Manipulation (EE 470), Lecture 20

import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path


# -----------------------------
# Helpers
# -----------------------------
def to_rgb(img_bgr):
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

def add_noise_sp(img, amount=0.01):
    """Salt & pepper noise: amount is the fraction of pixels to corrupt."""
    noisy = img.copy()
    h, w = noisy.shape[:2]
    num = int(amount * h * w)
    # salt
    coords = (np.random.randint(0, h, num), np.random.randint(0, w, num))
    noisy[coords] = 255
    # pepper
    coords = (np.random.randint(0, h, num), np.random.randint(0, w, num))
    noisy[coords] = 0
    return noisy

def add_noise_gaussian(img, sigma=25):
    """Add zero-mean Gaussian noise with std=sigma."""
    noise = np.random.normal(0, sigma, img.shape).astype(np.float32)
    noisy = img.astype(np.float32) + noise
    return np.clip(noisy, 0, 255).astype(np.uint8)

# -----------------------------
# Load
# -----------------------------
script_dir = Path(__file__).parent
image_path = script_dir / 'rubiks.jpg'
img = cv2.imread(str(image_path))
if img is None:
    raise FileNotFoundError("Could not find 'rubiks.jpg' in the working directory.")
img_rgb = to_rgb(img)

# -----------------------------
# Make noisy inputs
# -----------------------------
kernel_size = 5  # use odd sizes for median/gaussian
noisy_sp  = add_noise_sp(img, amount=0.01)
noisy_gau = add_noise_gaussian(img, sigma=25)

# -----------------------------
# Filters on S&P noise (primary comparison)
# -----------------------------
mean_blur     = cv2.blur(noisy_sp, (kernel_size, kernel_size))

# Box filter (normalized = mean). Unnormalized shows effect of raw sum; left here for completeness.
box_norm      = cv2.boxFilter(noisy_sp, ddepth=-1, ksize=(kernel_size, kernel_size), normalize=True)
box_unnorm    = cv2.boxFilter(noisy_sp, ddepth=-1, ksize=(kernel_size, kernel_size), normalize=False)

# Custom weighted kernel (center-weighted 3x3 as an example)
w_k = np.array([[1, 2, 1],
                [2, 4, 2],
                [1, 2, 1]], dtype=np.float32)
w_k /= w_k.sum()
weighted_3x3  = cv2.filter2D(noisy_sp, ddepth=-1, kernel=w_k)

gauss_blur    = cv2.GaussianBlur(noisy_sp, (kernel_size, kernel_size), sigmaX=0)
median_blur   = cv2.medianBlur(noisy_sp, kernel_size)
bilateral     = cv2.bilateralFilter(noisy_sp, d=9, sigmaColor=75, sigmaSpace=75)

# Laplacian response and a simple Laplacian-based sharpening
lap64         = cv2.Laplacian(noisy_sp, ddepth=cv2.CV_64F, ksize=3)
lap_abs       = cv2.convertScaleAbs(lap64)
# Unsharp via Laplacian: sharpen = img - alpha * Laplacian
alpha = 1.0
sharp_lap     = cv2.addWeighted(noisy_sp, 1.0, -lap_abs, alpha, 0)

# -----------------------------
# Optional: Filters on Gaussian noise (quick glance)
# -----------------------------
median_gau    = cv2.medianBlur(noisy_gau, kernel_size)
gauss_gau     = cv2.GaussianBlur(noisy_gau, (kernel_size, kernel_size), 0)
bilat_gau     = cv2.bilateralFilter(noisy_gau, d=9, sigmaColor=75, sigmaSpace=75)

# -----------------------------
# Show
# -----------------------------
rows = 3
cols = 4
fig, axs = plt.subplots(rows, cols, figsize=(14, 9))
fig.suptitle("Filter Comparison on Salt & Pepper Noise (primary) + Gaussian noise (optional)", y=0.98)

def show(ax, img_bgr, title):
    ax.imshow(to_rgb(img_bgr))
    ax.set_title(title, fontsize=9)
    ax.axis("off")

# Row 1: originals and noisy inputs
show(axs[0,0], img,      "Original")
show(axs[0,1], noisy_sp, "Salt & Pepper")
show(axs[0,2], noisy_gau,"Gaussian Noise")
show(axs[0,3], mean_blur, f"cv2.blur ({kernel_size}x{kernel_size})")

# Row 2: box/weighted/gaussian/median
show(axs[1,0], box_norm,   "cv2.boxFilter (normalized)")
show(axs[1,1], box_unnorm, "cv2.boxFilter (unnormalized)")
show(axs[1,2], weighted_3x3,"cv2.filter2D (weighted 3x3)")
show(axs[1,3], gauss_blur, f"cv2.GaussianBlur ({kernel_size}x{kernel_size})")

# Row 3: median/bilateral/lap/sharpen
show(axs[2,0], median_blur, f"cv2.medianBlur (ksize={kernel_size})")
show(axs[2,1], bilateral,    "cv2.bilateralFilter (d=9,75,75)")
show(axs[2,2], lap_abs,      "cv2.Laplacian (abs)")
show(axs[2,3], sharp_lap,    "Sharpen via Laplacian")

plt.tight_layout()
plt.show()

# Extra peek: filters on Gaussian-noisy image (comment out if not needed)
fig2, axs2 = plt.subplots(1, 3, figsize=(10, 3.2))
show(axs2[0], median_gau,  "Median on Gaussian noise")
show(axs2[1], gauss_gau,   "Gaussian on Gaussian noise")
show(axs2[2], bilat_gau,   "Bilateral on Gaussian noise")
plt.tight_layout()
plt.show()
