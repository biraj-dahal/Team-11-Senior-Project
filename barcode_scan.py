# Database of items with their attributes
ITEMS_DATABASE = {
    "123456": {"name": "Milk", "perishable": True, "fragile": False, "hazardous": False},
    "654321": {"name": "Glass Vase", "perishable": False, "fragile": True, "hazardous": False},
    "789012": {"name": "Cleaning Chemicals", "perishable": False, "fragile": False, "hazardous": True},
    "345678": {"name": "Eggs", "perishable": True, "fragile": True, "hazardous": False},
}

def scan_barcode(barcode):
    """
    Simulates scanning a barcode and classifying the item.
    """
    # Check if barcode exists in the database
    item = ITEMS_DATABASE.get(barcode)
    
    if not item:
        return f"Error: Item with barcode {barcode} not found."
    
    # Get item attributes
    name = item['name']
    attributes = []
    if item['perishable']:
        attributes.append("Perishable")
    if item['fragile']:
        attributes.append("Fragile")
    if item['hazardous']:
        attributes.append("Hazardous")
    
    # Build response
    classification = ", ".join(attributes) if attributes else "No special classification"
    return f"Item: {name}\nClassification: {classification}"

# Example Usage
if __name__ == "__main__":
    print("Welcome to the Package Scanner!")
    while True:
        barcode = input("Enter the barcode to scan (or type 'exit' to quit): ").strip()
        if barcode.lower() == "exit":
            print("Exiting the scanner. Goodbye!")
            break
        print(scan_barcode(barcode))
