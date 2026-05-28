#!/usr/bin/env python3
import argparse
import os
import struct


def c_string(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


def c_float(value):
    text = f"{value:.9g}"
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return text + "f"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    args = parser.parse_args()

    with open(args.checkpoint, "rb") as f:
        data = f.read()

    if len(data) % 4 != 0:
        raise SystemExit(f"{args.checkpoint}: size is not a multiple of 4 bytes")

    floats = list(struct.unpack("<" + "f" * (len(data) // 4), data))
    padded = floats + [0.0] * 7
    checkpoint = os.path.abspath(args.checkpoint)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="ascii") as f:
        f.write("#ifndef LINE_FOLLOW_MODEL_WEIGHTS_H\n")
        f.write("#define LINE_FOLLOW_MODEL_WEIGHTS_H\n\n")
        f.write(f"#define LINE_FOLLOW_MODEL_RAW_FLOATS {len(floats)}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_PADDED_FLOATS {len(padded)}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_BYTES {len(data)}\n\n")
        f.write(
            'static const char line_follow_model_checkpoint[] = "'
            + c_string(checkpoint)
            + '";\n\n'
        )
        f.write("static const float line_follow_model_weights[LINE_FOLLOW_MODEL_PADDED_FLOATS] = {\n")
        for i, value in enumerate(padded):
            if i % 4 == 0:
                f.write("  ")
            f.write(c_float(value))
            if i != len(padded) - 1:
                f.write(", ")
            if i % 4 == 3:
                f.write("\n")
        if len(padded) % 4 != 0:
            f.write("\n")
        f.write("};\n\n")
        f.write("#endif\n")


if __name__ == "__main__":
    main()
