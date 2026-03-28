import torch
import requests
from PIL import Image
from transformers import AutoProcessor, PaliGemmaForConditionalGeneration

model = PaliGemmaForConditionalGeneration.from_pretrained(
    "google/paligemma2-3b-mix-224",
    dtype=torch.bfloat16,
    device_map="auto",
    attn_implementation="sdpa",
)
processor = AutoProcessor.from_pretrained(
    "google/paligemma2-3b-mix-224",
)

prompt = "<image> detect dinosaur"
url = "https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/pipeline-cat-chonk.jpeg"
# image = Image.open(requests.get(url, stream=True).raw)
image = Image.open("/tmp/vlm_snapshot/rgb.png")
inputs = processor(image, prompt, return_tensors="pt").to(model.device)

output = model.generate(**inputs, max_new_tokens=50, cache_implementation="static")
print(processor.decode(output[0], skip_special_tokens=True))
