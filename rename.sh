#!/bin/bash

# Check if the correct number of arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 new_string"
    exit 1
fi

# New string to replace with
new_string=$1

camel_new_string=$(echo "$new_string" | sed -r 's/(^|_)([a-z])/\U\2/g')

# Function to rename files and directories
rename_files_and_directories() {
    find . -depth -name '*template_controller*' | while read -r file; do
        newfile=$(echo "$file" | sed "s/template_controller/${new_string}/g")
        mv "$file" "$newfile"
    done
    find . -depth -type d -name '*TemplateController*' | while read -r file; do
        newfile=$(echo "$file" | sed "s/TemplateController/${camel_new_string}/g")
        mv "$file" "$newfile"
    done
    find . -depth -type f -name '*TemplateController*' | while read -r file; do
        newfile=$(echo "$file" | sed "s/TemplateController/${camel_new_string}/g")
        mv "$file" "$newfile"
    done
}

# Function to replace text inside files
replace_text_inside_files() {
    find . -type f -not -path './.git/*' -exec sed -i "s/template_controller/${new_string}/g" {} +
    find . -type f -not -path './.git/*' -exec sed -i "s/TemplateController/${camel_new_string}/g" {} +
}

# Execute the functions
rename_files_and_directories
replace_text_inside_files

echo "Renaming and text replacement completed."
