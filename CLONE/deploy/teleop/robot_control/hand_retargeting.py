from dex_retargeting.retargeting_config import RetargetingConfig
from pathlib import Path
import yaml
from enum import Enum
import os

# Get path relative to this file
_current_dir = Path(__file__).parent.absolute()
_resources_dir = _current_dir.parent.parent / "resources"

class HandType(Enum):
    INSPIRE_HAND = str(_resources_dir / "inspire_hand" / "inspire_hand.yml")
    UNITREE_DEX3 = str(_resources_dir / "unitree_hand" / "unitree_dex3.yml")
    UNITREE_DEX3_Unit_Test = str(_resources_dir / "unitree_hand" / "unitree_dex3.yml")

class HandRetargeting:
    def __init__(self, hand_type: HandType):
        if hand_type == HandType.UNITREE_DEX3:
            RetargetingConfig.set_default_urdf_dir(str(_resources_dir))
        elif hand_type == HandType.UNITREE_DEX3_Unit_Test:
            RetargetingConfig.set_default_urdf_dir(str(_resources_dir))
        elif hand_type == HandType.INSPIRE_HAND:
            RetargetingConfig.set_default_urdf_dir(str(_resources_dir))

        config_file_path = Path(hand_type.value)

        try:
            with config_file_path.open('r') as f:
                self.cfg = yaml.safe_load(f)
                
            if 'left' not in self.cfg or 'right' not in self.cfg:
                raise ValueError("Configuration file must contain 'left' and 'right' keys.")

            left_retargeting_config = RetargetingConfig.from_dict(self.cfg['left'])
            right_retargeting_config = RetargetingConfig.from_dict(self.cfg['right'])
            self.left_retargeting = left_retargeting_config.build()
            self.right_retargeting = right_retargeting_config.build()
        
        except FileNotFoundError:
            print(f"Configuration file not found: {config_file_path}")
            raise
        except yaml.YAMLError as e:
            print(f"YAML error while reading {config_file_path}: {e}")
            raise
        except Exception as e:
            print(f"An error occurred: {e}")
            raise