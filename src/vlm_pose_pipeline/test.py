import re

import torch
import requests
from PIL import Image, ImageDraw
from transformers import AutoProcessor, PaliGemmaForConditionalGeneration


def parse_loc_tokens(text):
    matches = re.findall(r"<loc(\d{4})>", text)
    if len(matches) < 4:
        return None
    return [int(matches[0]), int(matches[1]), int(matches[2]), int(matches[3])]


def quantized_to_pixel(loc_value, image_size, bins=1024):
    loc_value = max(0, min(loc_value, bins - 1))
    return int(round((loc_value / float(bins - 1)) * (image_size - 1)))


def draw_detection(image, decoded_text):
    loc_values = parse_loc_tokens(decoded_text)
    if loc_values is None:
        print("No <loc####> tokens found in model output; skipping drawing")
        return image

    y_min, x_min, y_max, x_max = loc_values
    width, height = image.size

    u_min = quantized_to_pixel(x_min, width)
    u_max = quantized_to_pixel(x_max, width)
    v_min = quantized_to_pixel(y_min, height)
    v_max = quantized_to_pixel(y_max, height)

    u0 = max(0, min(u_min, u_max))
    u1 = min(width - 1, max(u_min, u_max))
    v0 = max(0, min(v_min, v_max))
    v1 = min(height - 1, max(v_min, v_max))

    annotated = image.copy()
    draw = ImageDraw.Draw(annotated)
    draw.rectangle([(u0, v0), (u1, v1)], outline=(0, 255, 0), width=3)

    label = decoded_text.split("<loc", 1)[0].strip() or "detected"
    draw.text((u0 + 4, max(0, v0 - 18)), label, fill=(0, 255, 0))

    print(f"BBox pixels: ({u0}, {v0}) - ({u1}, {v1})")
    return annotated


model = PaliGemmaForConditionalGeneration.from_pretrained(
    "google/paligemma2-3b-mix-224",
    dtype=torch.bfloat16,
    device_map="auto",
    attn_implementation="sdpa",
)
processor = AutoProcessor.from_pretrained(
    "google/paligemma2-3b-mix-224",
)

prompt = "<image> detect blue cylinder"
url = "https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/pipeline-cat-chonk.jpeg"
# image = Image.open(requests.get(url, stream=True).raw)
image = Image.open("/tmp/vlm_snapshot/rgb.png")
inputs = processor(image, prompt, return_tensors="pt").to(model.device)

output = model.generate(**inputs, max_new_tokens=50, cache_implementation="static")
decoded = processor.decode(output[0], skip_special_tokens=True)
print(decoded)

annotated = draw_detection(image, decoded)
annotated_path = "/tmp/vlm_snapshot/rgb_annotated.png"
annotated.save(annotated_path)
print(f"Saved annotated image to: {annotated_path}")

try:
    annotated.show()
except Exception as exc:
    print(f"Could not open image viewer automatically: {exc}")
