import json
import pathlib

class ProtocolManager:
    def __init__(self, json_path, intersection_file_path):
        self.json_path = pathlib.Path(json_path)
        self.intersection_file_path = intersection_file_path
        self._load()

        self.name_map = {
            # Medications
            "med_r1": "Morning Med: Droxidopa and CARB at 7:00 AM",
            "med_r2": "Morning Med: Medodrine at 10:00 AM",
            "med_r3": "Afternoon Med: Droxidopa and CARB at 1:00 PM",
            "med_r4": "Afternoon Med: Droxidopa and CARB at 5:00 PM",

            # Meals
            "food_r1": "Breakfast at 9:00 AM",
            "food_r2": "Lunch at 12:00 PM",
            "food_r3": "Meal at 4:00 PM",
            "food_r4": "Dinner at 7:00 PM"
        }

    def _load(self):
        with open(self.json_path) as f:
            self.data = json.load(f)

    def save(self):
        with open(self.json_path, "w") as f:
            json.dump(self.data, f, indent=2)

    def get_as_string(self):
        return json.dumps(self.data)
    
    def update_protocol_json(self):
        """
        Reads an intersection.txt file and updates the 'provided' field of
        the JSON protocols.  name_map maps short IDs (e.g. 'med_r1') to
        full protocol names (e.g. 'Morning Medication at 9:00am').
        """
        self._load()
        # 1) collect the list of short names found in the file
        found_ids = set()
        with open(self.intersection_file_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    short_name = parts[1]
                    found_ids.add(short_name)

        # 2) map the short names to canonical protocol names
        active_protocol_names = set()
        for short_name in found_ids:
            full_name = self.name_map.get(short_name)
            if full_name:
                active_protocol_names.add(full_name)

        # 3) update self.data
        for item in self.data:
            if item.get("protocol") in active_protocol_names:
                item["provided"] = True
            else:
                item["provided"] = False

        # 4) save the updated file
        self.save()
    def update_confirmed(self, protocol_name, confirmed):
        """Match the most similar protocol entry and update 'confirmed' field."""
        for item in self.data:
            if protocol_name.lower() in item["protocol"].lower():
                item["confirmed"] = confirmed
                break
        self.save()
