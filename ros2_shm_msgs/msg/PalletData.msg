# 载具类型定义（替代 perception_idl/PalletData）
# PalletType: 0=UNKNOWN 1=CHUAN_SHAPED_WOOD_PALLET 2=TIAN_SHAPED_WOOD_PALLET 3=CHUAN_SHAPED_PLASTIC_PALLET 4=TIAN_SHAPED_PLASTIC_PALLET 5=CHUAN_SHAPED_META_PALLET 6=TIAN_SHAPED_META_PALLET 7=PLASTAIC_CONTAINER 8=WHEELED_META_CAGE 9=PLASTIC_BOX_PALLET
uint32 id
uint8 pallet_type
bool loaded
geometry_msgs/Point pallet_position
float32 confidence
shm_msgs/EulerAngle pallet_orientation
geometry_msgs/Vector3 pallet_size
geometry_msgs/Point cargo_position
geometry_msgs/Vector3 cargo_size
bool on_fork_status