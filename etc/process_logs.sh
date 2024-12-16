#!/bin/bash

# Usage: keys are preset, only need to input the raw log and it will output the processed one in the same place

# check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <input_log1> [<input_log2> ...]"
    exit 1
fi

if ! command -v mc_bin_utils &> /dev/null; then
    echo "Error: mc_bin_utils is not installed."
    exit 1
fi

# keys to keep in reprocessed log
keys=("XsensPlugin_raw_*" "ForceShoes_*")

for input_log in "$@"; do
    echo "Processing ${input_log}"
    name_bin=$input_log
    name_only="${name_bin%.*}" #remove extension for output name
    mc_bin_utils extract "$input_log" "${name_only}_processed.bin" --keys "${keys[@]}"
    echo "Created ${name_only}_processed.bin"
done

echo "Processed all logs."
