import xml.etree.ElementTree as ET
import os
import argparse

def extract_src_path(full_path):
    """Extracts the path starting from /src/."""
    src_index = full_path.find('/src/')
    return full_path[src_index:] if src_index != -1 else full_path

def sort_key(error_element):
    location_element = error_element.find('location')
    file_attr = location_element.get('file', '') if location_element is not None else ''
    # Use the helper function to extract the path from /src/
    file_attr = extract_src_path(file_attr)
    severity_attr = error_element.get('severity', '')
    id_attr = error_element.get('id', '')
    return (file_attr, severity_attr, id_attr)

def sort_cppcheck_report(input_filepath):
    # ... existing code ...
    try:
        tree = ET.parse(input_filepath)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file {input_filepath}: {e}")
        return None
    except FileNotFoundError:
        print(f"Input file not found: {input_filepath}")
        return None
    errors_container = root.find('errors')
    if errors_container is None:
        print("No <errors> tag found in the XML. Assuming no errors to sort.")
        return root
    error_elements = list(errors_container.findall('error'))
    error_elements.sort(key=sort_key)
    for error_el in list(errors_container.findall('error')): 
        errors_container.remove(error_el)
    for error_el in error_elements:
        errors_container.append(error_el)
    print(f"Successfully sorted report from {input_filepath} in memory.")
    return root

def get_error_signature(error_element):
    """Generates a unique signature for an error element for diffing purposes."""
    sig_id = error_element.get('id', '')
    sig_severity = error_element.get('severity', '')
    sig_msg = error_element.get('msg', '')
    loc_file = ''
    location_element = error_element.find('location') # Primary location
    if location_element is not None:
        loc_file = location_element.get('file', '')
        # Use the helper function to extract the path from /src/
        loc_file = extract_src_path(loc_file)
    return (sig_id, sig_severity, sig_msg, loc_file)

def diff_cppcheck_reports(ordered_report_root_element, baseline_report_path, diff_output_path):
    # ... existing code ...
    if ordered_report_root_element is None:
        print(f"Input sorted report data is None. Cannot perform diff.")
        diff_root_empty = ET.Element("results", version="2")
        ET.SubElement(diff_root_empty, "cppcheck", version="unknown_due_to_input_error")
        ET.SubElement(diff_root_empty, "errors")
        diff_tree_empty = ET.ElementTree(diff_root_empty)
        try:
            output_dir = os.path.dirname(diff_output_path)
            if output_dir and not os.path.exists(output_dir):
                 os.makedirs(output_dir)
            diff_tree_empty.write(diff_output_path, encoding='UTF-8', xml_declaration=True)
            print(f"Empty diff report created at: {diff_output_path} as input report data was invalid.")
        except IOError as e_write:
            print(f"Error writing empty diff report to {diff_output_path}: {e_write}")
        return
    cppcheck_version_element = ordered_report_root_element.find('cppcheck')
    cppcheck_version_str = cppcheck_version_element.get('version') if cppcheck_version_element is not None else "unknown"
    baseline_error_signatures = set()
    try:
        baseline_tree = ET.parse(baseline_report_path)
        baseline_root = baseline_tree.getroot()
        baseline_errors_container = baseline_root.find('errors')
        if baseline_errors_container is not None:
            for error_el in baseline_errors_container.findall('error'):
                baseline_error_signatures.add(get_error_signature(error_el))
    except FileNotFoundError:
        print(f"Baseline file not found: {baseline_report_path}. Assuming no baseline errors.")
    except ET.ParseError as e_parse_baseline:
        print(f"Error parsing baseline XML file {baseline_report_path}: {e_parse_baseline}. Assuming no baseline errors.")
    new_errors = []
    ordered_errors_container = ordered_report_root_element.find('errors')
    if ordered_errors_container is not None:
        for error_el in ordered_errors_container.findall('error'):
            if get_error_signature(error_el) not in baseline_error_signatures:
                new_errors.append(error_el)
    diff_root = ET.Element("results", version="2")
    ET.SubElement(diff_root, "cppcheck", version=cppcheck_version_str)
    diff_errors_container = ET.SubElement(diff_root, "errors")
    for error_el in new_errors:
        # Modify all location paths to start from /src/
        for location_element in error_el.findall('location'):
            file_attr = location_element.get('file', '')
            location_element.set('file', extract_src_path(file_attr))
        diff_errors_container.append(error_el)
    try:
        output_dir = os.path.dirname(diff_output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        diff_tree = ET.ElementTree(diff_root)
        diff_tree.write(diff_output_path, encoding='UTF-8', xml_declaration=True)
        print(f"Successfully generated diff report: {diff_output_path}. Found {len(new_errors)} new error(s).")
    except IOError as e_write_diff:
        print(f"Error writing diff report to {diff_output_path}: {e_write_diff}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_diff_output_dir = os.path.join(script_dir, "cppcheck_results")
    default_diff_output_path = os.path.join(default_diff_output_dir, "show_diff.xml")

    parser = argparse.ArgumentParser(description="Sorts a Cppcheck report and diffs it against a baseline.")
    parser.add_argument("current_report_path", help="Path to the current Cppcheck XML report.")
    parser.add_argument("baseline_report_path", help="Path to the baseline Cppcheck XML report.")
    parser.add_argument("--diff_output_path", 
                        help=f"Optional: Path to save the generated diff XML report. Defaults to '{default_diff_output_path}'", 
                        default=default_diff_output_path)

    args = parser.parse_args()

    # Ensure the directory for the default diff output path exists if it's being used
    actual_diff_output_dir = os.path.dirname(args.diff_output_path)
    if actual_diff_output_dir and not os.path.exists(actual_diff_output_dir):
        os.makedirs(actual_diff_output_dir)
        print(f"Created directory for diff output: {actual_diff_output_dir}")

    print("-" * 50)
    print(f"Step 1: Sorting Cppcheck report '{args.current_report_path}' in memory")
    sorted_report_root = sort_cppcheck_report(args.current_report_path)
    
    if sorted_report_root is None:
        print("Sorting failed or original report is invalid. Aborting diff operation.")
        print(f"Attempting to create an empty diff report at: {args.diff_output_path}")
        diff_cppcheck_reports(None, args.baseline_report_path, args.diff_output_path)
    else:
        print("Sorting in memory completed.")
        print("-" * 50)
        print(f"Step 2: Comparing sorted report with baseline '{args.baseline_report_path}'")
        print(f"Diff output will be saved to: {args.diff_output_path}")
        
        diff_cppcheck_reports(sorted_report_root, args.baseline_report_path, args.diff_output_path)
        print("Diff operation completed.")

    print("-" * 50)
    print("Processing finished.")
