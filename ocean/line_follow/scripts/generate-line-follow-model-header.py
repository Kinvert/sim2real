#!/usr/bin/env python3
import argparse
import os
import struct


DEFAULT_FIXED_SCALE = 16384


def c_string(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


def c_float(value):
    text = f"{value:.9g}"
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return text + "f"


def align8(index):
    return (index + 7) & ~7


def compact_float_count(obs_size, hidden_size, num_layers, num_actions):
    decoder_size = num_actions + 1
    return (
        hidden_size * obs_size
        + decoder_size * hidden_size
        + num_actions
        + num_layers * 3 * hidden_size * hidden_size
    )


def storage_float_count(obs_size, hidden_size, num_layers, num_actions):
    decoder_size = num_actions + 1
    index = 0
    index = align8(index)
    index += hidden_size * obs_size
    index = align8(index)
    index += decoder_size * hidden_size
    index = align8(index)
    index += num_actions
    for _ in range(num_layers):
        index = align8(index)
        index += 3 * hidden_size * hidden_size
    return index


def aligned_read_float_count(obs_size, hidden_size, num_layers, num_actions):
    return align8(
        storage_float_count(obs_size, hidden_size, num_layers, num_actions)
    )


def checkpoint_to_aligned(values, obs_size, hidden_size, num_layers, num_actions):
    storage_count = storage_float_count(
        obs_size, hidden_size, num_layers, num_actions
    )
    aligned_count = aligned_read_float_count(
        obs_size, hidden_size, num_layers, num_actions
    )
    if len(values) not in (storage_count, aligned_count):
        raise SystemExit(
            f"checkpoint has {len(values)} floats; expected native storage "
            f"{storage_count} or aligned read {aligned_count}"
        )
    return values + [0.0] * (aligned_count - len(values))


def take_aligned(values, index, count):
    end = index + count
    if end > len(values):
        raise SystemExit(
            f"checkpoint ended while reading {count} floats at aligned index {index}"
        )
    return values[index:end], align8(end)


def fold_action_weights(floats, obs_size, hidden_size, num_layers, num_actions):
    if num_layers != 0:
        return None

    decoder_size = num_actions + 1
    index = 0
    encoder, index = take_aligned(floats, index, hidden_size * obs_size)
    decoder, index = take_aligned(floats, index, decoder_size * hidden_size)
    _log_std, _index = take_aligned(floats, index, num_actions)

    folded = []
    for action in range(num_actions):
        for obs in range(obs_size):
            total = 0.0
            for hidden in range(hidden_size):
                total += (
                    decoder[action * hidden_size + hidden]
                    * encoder[hidden * obs_size + obs]
                )
            folded.append(total)
    return folded


def c_int_list(values):
    return [str(int(value)) for value in values]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("output")
    parser.add_argument("--obs-size", type=int, default=3)
    parser.add_argument("--hidden-size", type=int, default=8)
    parser.add_argument("--num-layers", type=int, default=0)
    parser.add_argument("--num-actions", type=int, default=2)
    parser.add_argument("--fixed-scale", type=int, default=DEFAULT_FIXED_SCALE)
    args = parser.parse_args()

    with open(args.checkpoint, "rb") as f:
        data = f.read()

    if len(data) % 4 != 0:
        raise SystemExit(f"{args.checkpoint}: size is not a multiple of 4 bytes")

    floats = list(struct.unpack("<" + "f" * (len(data) // 4), data))
    checkpoint = os.path.abspath(args.checkpoint)
    expected = storage_float_count(
        args.obs_size, args.hidden_size, args.num_layers, args.num_actions
    )
    aligned_read = aligned_read_float_count(
        args.obs_size, args.hidden_size, args.num_layers, args.num_actions
    )
    compact = compact_float_count(
        args.obs_size, args.hidden_size, args.num_layers, args.num_actions
    )
    if len(floats) not in (expected, aligned_read):
        raise SystemExit(
            f"{args.checkpoint}: raw_floats={len(floats)} expected_native={expected} "
            f"aligned_read={aligned_read} compact={compact} "
            f"hidden_size={args.hidden_size} num_layers={args.num_layers}. "
            "Retrain/export this architecture with padded native checkpoint support."
        )

    # Native PufferLib checkpoints store the aligned parameter buffer prefix.
    # Add only the harmless final tail zeros that get_weights_aligned may touch.
    padded = checkpoint_to_aligned(
        floats,
        args.obs_size,
        args.hidden_size,
        args.num_layers,
        args.num_actions,
    )
    padded_q = [round(value * args.fixed_scale) for value in padded]
    folded = None
    folded = fold_action_weights(
        padded,
        args.obs_size,
        args.hidden_size,
        args.num_layers,
        args.num_actions,
    )
    folded_q = []
    if folded is not None:
        folded_q = [round(value * args.fixed_scale) for value in folded]

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="ascii") as f:
        f.write("#ifndef LINE_FOLLOW_MODEL_WEIGHTS_H\n")
        f.write("#define LINE_FOLLOW_MODEL_WEIGHTS_H\n\n")
        f.write(f"#define LINE_FOLLOW_MODEL_RAW_FLOATS {len(floats)}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_PADDED_FLOATS {len(padded)}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_BYTES {len(data)}\n\n")
        f.write(f"#define LINE_FOLLOW_MODEL_OBS_SIZE {args.obs_size}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_HIDDEN_SIZE {args.hidden_size}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_NUM_LAYERS {args.num_layers}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_NUM_ACTIONS {args.num_actions}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_EXPECTED_RAW_FLOATS {expected}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_ALIGNED_READ_FLOATS {aligned_read}\n")
        f.write(f"#define LINE_FOLLOW_MODEL_FIXED_SCALE {args.fixed_scale}\n")
        f.write(
            f"#define LINE_FOLLOW_MODEL_FOLDED_SUPPORTED "
            f"{1 if folded is not None else 0}\n\n"
        )
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
        f.write("static const int line_follow_model_weights_q[LINE_FOLLOW_MODEL_PADDED_FLOATS] = {\n")
        for i, value in enumerate(c_int_list(padded_q)):
            if i % 4 == 0:
                f.write("  ")
            f.write(value)
            if i != len(padded_q) - 1:
                f.write(", ")
            if i % 4 == 3:
                f.write("\n")
        if len(padded_q) % 4 != 0:
            f.write("\n")
        f.write("};\n\n")
        if folded is not None:
            f.write(
                "static const float line_follow_model_folded_action_w"
                "[LINE_FOLLOW_MODEL_NUM_ACTIONS * LINE_FOLLOW_MODEL_OBS_SIZE] = {\n"
            )
            for i, value in enumerate(folded):
                if i % args.obs_size == 0:
                    f.write("  ")
                f.write(c_float(value))
                if i != len(folded) - 1:
                    f.write(", ")
                if i % args.obs_size == args.obs_size - 1:
                    f.write("\n")
            f.write("};\n\n")
            f.write(
                "static const int line_follow_model_folded_action_w_q"
                "[LINE_FOLLOW_MODEL_NUM_ACTIONS * LINE_FOLLOW_MODEL_OBS_SIZE] = {\n"
            )
            for i, value in enumerate(c_int_list(folded_q)):
                if i % args.obs_size == 0:
                    f.write("  ")
                f.write(value)
                if i != len(folded_q) - 1:
                    f.write(", ")
                if i % args.obs_size == args.obs_size - 1:
                    f.write("\n")
            f.write("};\n\n")
        else:
            f.write("static const float line_follow_model_folded_action_w[1] = {0.0f};\n")
            f.write("static const int line_follow_model_folded_action_w_q[1] = {0};\n\n")
        f.write("#endif\n")


if __name__ == "__main__":
    main()
