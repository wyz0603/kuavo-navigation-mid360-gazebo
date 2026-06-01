import xml.etree.ElementTree as ET
import os
import argparse # Added for command-line arguments

def sort_key(error_element):
    """Defines the sorting key for an error element."""
    # First criterion: 'file' attribute of the first 'location' child
    location_element = error_element.find('location')
    file_attr = location_element.get('file', '') if location_element is not None else ''

    # Second criterion: 'severity' attribute of the error element
    severity_attr = error_element.get('severity', '')

    # Third criterion: 'id' attribute of the error element
    id_attr = error_element.get('id', '')

    return (file_attr, severity_attr, id_attr)

def sort_cppcheck_report(input_filepath, output_filepath):
    """
    Sorts the <error> elements in a Cppcheck XML report and writes the result.

    Args:
        input_filepath (str): Path to the input Cppcheck XML report.
        output_filepath (str): Path to save the sorted Cppcheck XML report.
    """
    try:
        tree = ET.parse(input_filepath)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file {input_filepath}: {e}")
        return
    except FileNotFoundError:
        print(f"Input file not found: {input_filepath}")
        return

    errors_container = root.find('errors')
    if errors_container is None:
        print("No <errors> tag found in the XML. Assuming no errors to sort.")
        # Write the original tree or an empty one if preferred
        try:
            tree.write(output_filepath, encoding='UTF-8', xml_declaration=True)
            print(f"Original report (or empty) saved to: {output_filepath} as no <errors> tag was found.")
        except IOError as e:
            print(f"Error writing to output file {output_filepath}: {e}")
        return

    # Get all error elements
    error_elements = list(errors_container.findall('error'))

    # Sort the error elements
    error_elements.sort(key=sort_key)

    # Remove existing error elements from the container
    for error_el in errors_container.findall('error'): # No need to copy list for findall before remove
        errors_container.remove(error_el)

    # Add sorted error elements back to the container
    for error_el in error_elements:
        errors_container.append(error_el)

    # Write the modified tree to the output file
    try:
        # Ensure the output directory exists
        output_dir_for_file = os.path.dirname(output_filepath)
        if output_dir_for_file and not os.path.exists(output_dir_for_file):
            os.makedirs(output_dir_for_file)
            print(f"Created directory for output: {output_dir_for_file}")
            
        tree.write(output_filepath, encoding='UTF-8', xml_declaration=True)
        print(f"Successfully sorted report saved to: {output_filepath}")
    except IOError as e:
        print(f"Error writing to output file {output_filepath}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sorts Cppcheck XML report errors and saves the result.")
    parser.add_argument("input_file", help="Path to the input Cppcheck XML report.")
    parser.add_argument("output_file", help="Path to save the sorted Cppcheck XML report.")

    args = parser.parse_args()

    sort_cppcheck_report(args.input_file, args.output_file) 