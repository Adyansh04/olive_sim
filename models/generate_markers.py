#!/usr/bin/env python3
"""
Generate placeholder WhyCode marker images
These are simple black and white patterns that users can replace with actual WhyCode markers
"""

try:
    from PIL import Image, ImageDraw, ImageFont
    import os
    
    def create_whycode_marker(marker_id, size=200):
        """Create a simple black and white marker with ID"""
        # Create white background
        img = Image.new('RGB', (size, size), 'white')
        draw = ImageDraw.Draw(img)
        
        # Draw black border
        border_width = 10
        draw.rectangle([0, 0, size-1, size-1], outline='black', width=border_width)
        
        # Draw marker ID in center (simplified pattern)
        # Create a simple pattern based on marker ID
        pattern_size = size // 5
        for i in range(3):
            for j in range(3):
                # Create a pattern based on marker_id and position
                if (marker_id + i + j) % 2 == 0:
                    x1 = border_width + pattern_size + i * pattern_size
                    y1 = border_width + pattern_size + j * pattern_size
                    x2 = x1 + pattern_size
                    y2 = y1 + pattern_size
                    draw.rectangle([x1, y1, x2, y2], fill='black')
        
        # Draw marker number in corner
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 30)
        except:
            font = ImageFont.load_default()
        
        draw.text((size - 40, 15), str(marker_id), fill='black', font=font)
        
        return img
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Generate markers 0-3
    for i in range(4):
        img = create_whycode_marker(i)
        output_path = os.path.join(script_dir, f'whycode_{i}', 'materials', 'textures', f'marker_{i}.png')
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        img.save(output_path)
        print(f"Created marker image: {output_path}")
    
    print("\nPlaceholder WhyCode markers created successfully!")
    print("These are simple patterns for testing. Replace with actual WhyCode markers for production use.")

except ImportError:
    print("PIL (Pillow) not available. Creating placeholder README instead.")
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for i in range(4):
        readme_path = os.path.join(script_dir, f'whycode_{i}', 'materials', 'textures', 'README.txt')
        os.makedirs(os.path.dirname(readme_path), exist_ok=True)
        with open(readme_path, 'w') as f:
            f.write(f"""WhyCode Marker {i} Texture

Place your marker_{i}.png file in this directory.

The marker image should be:
- Format: PNG
- Recommended size: 200x200 pixels or higher
- Pattern: High-contrast black and white QR-style pattern
- Unique pattern for marker ID {i}

To generate WhyCode markers, use a QR code generator or WhyCode marker generation tool.
""")
        print(f"Created README for marker {i}")
