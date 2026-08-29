import os
import sys
import base64
import zlib

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 obfuscate_configs.py <submit_dir>")
        sys.exit(1)
        
    submit_dir = sys.argv[1]
    
    # Files to obfuscate
    target_files = ["config.yaml", "steer_gain.yaml", "ref_vel.yaml"]
    
    print(f"Obfuscating configuration files in {submit_dir}...")
    
    # 1. Find and obfuscate target yaml files
    for root, dirs, files in os.walk(submit_dir):
        for file in files:
            if file in target_files:
                yaml_path = os.path.join(root, file)
                enc_path = yaml_path.replace('.yaml', '.enc')
                
                with open(yaml_path, 'r') as f:
                    data = f.read().encode('utf-8')
                    
                # Obfuscate: compress then base85 encode
                enc_data = base64.b85encode(zlib.compress(data))
                
                with open(enc_path, 'wb') as f:
                    f.write(enc_data)
                    
                # Remove original yaml file
                os.remove(yaml_path)
                print(f"  Obfuscated {yaml_path} -> {enc_path}")

    # 2. Update launch files to point to .enc instead of .yaml
    for root, dirs, files in os.walk(submit_dir):
        for file in files:
            if file.endswith('.xml') or file.endswith('.py'):
                file_path = os.path.join(root, file)
                try:
                    with open(file_path, 'r') as f:
                        content = f.read()
                    
                    # Replace yaml references
                    new_content = content
                    for target in target_files:
                        new_content = new_content.replace(target, target.replace('.yaml', '.enc'))
                    
                    if content != new_content:
                        with open(file_path, 'w') as f:
                            f.write(new_content)
                        print(f"  Updated references in {file_path}")
                except Exception as e:
                    # Ignore non-text files or other errors
                    pass
    
    print("Obfuscation complete!")

if __name__ == "__main__":
    main()
