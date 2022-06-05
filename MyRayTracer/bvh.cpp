#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}




BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

/* Sorting predicates for objects based on the centroid coordinates and a specific axis */
bool sort_X(Object* o1, Object* o2) { return o1->getCentroid().x < o2->getCentroid().x; }
bool sort_Y(Object* o1, Object* o2) { return o1->getCentroid().y < o2->getCentroid().y; }
bool sort_Z(Object* o1, Object* o2) { return o1->getCentroid().z < o2->getCentroid().z; }

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   
	// Check if the number of objects in the node is fewer than the threshold
	int n_objects = right_index - left_index;
	if (n_objects <= Threshold) {
		node->makeLeaf(left_index, n_objects);
		return;
	}


	// Find the axis with the largest range of centroids
	double max_x = -FLT_MAX, min_x = FLT_MAX;
	double max_y = -FLT_MAX, min_y = FLT_MAX;
	double max_z = -FLT_MAX, min_z = FLT_MAX;
	for (int i=left_index; i<right_index; i++) {
		double x = objects[i]->getCentroid().x;
		double y = objects[i]->getCentroid().y;
		double z = objects[i]->getCentroid().z;

		max_x = (x > max_x) ? x : max_x;
		max_y = (y > max_y) ? y : max_y;
		max_z = (z > max_z) ? z : max_z;
		
		min_x = (x < min_x) ? x : min_x;
		min_y = (y < min_y) ? y : min_y;
		min_z = (z < min_z) ? z : min_z;
	}

	double dif_x = max_x - min_x;
	double dif_y = max_y - min_y;
	double dif_z = max_z - min_z;
	
	int axis = (dif_x >= dif_y) ? 0 : (dif_y >= dif_z) ? 1 : 2; // x=0, y=1, z=2


	// Sort the objects by their centroid in the calculated axis
	if (axis == 0) { 
		sort(objects.begin() + left_index, objects.begin() + right_index, sort_X);
	}
	else if (axis == 1) { 
		sort(objects.begin() + left_index, objects.begin() + right_index, sort_Y);
	}
	else if (axis == 2) {
		sort(objects.begin() + left_index, objects.begin() + right_index, sort_Z);
	}
	
	int split_index = n_objects / 2 + left_index;
	int n_nodes = nodes.size();


	// Create the child nodes
	BVHNode *left_node = new BVHNode();
	BVHNode *right_node = new BVHNode();

	// Add them to the node vector
	nodes.push_back(left_node);
	nodes.push_back(right_node);

	// Save the index to the left node in the parent node
	node->makeNode(n_nodes);

	// Define their bounding boxes
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB left_box = AABB(min, max);
	AABB right_box = AABB(min, max);

	for (int i = left_index; i < split_index; i++) {
		AABB bbox = objects[i]->GetBoundingBox();
		left_box.extend(bbox);
	}
	for (int i = split_index; i < right_index; i++) {
		AABB bbox = objects[i]->GetBoundingBox();
		right_box.extend(bbox);
	}

	left_box.min.x -= EPSILON; left_box.min.y -= EPSILON; left_box.min.z -= EPSILON;
	left_box.max.x += EPSILON; left_box.max.y += EPSILON; left_box.max.z += EPSILON;
	right_box.min.x -= EPSILON; right_box.min.y -= EPSILON; right_box.min.z -= EPSILON;
	right_box.max.x += EPSILON; right_box.max.y += EPSILON; right_box.max.z += EPSILON;

	left_node->setAABB(left_box);
	right_node->setAABB(right_box);


	// Call the recursive function for the child nodes
	build_recursive(left_index, split_index, left_node);
	build_recursive(split_index, right_index, right_node);	
		
	}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float t1, t2;
			bool hit1=false, hit2;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			Object* closestObj = NULL;

			BVHNode* currentNode = nodes[0];

			// Intersect with the world box
			hit1 = currentNode->getAABB().intercepts(ray, t1);
			if (!hit1) return false;

			while(true) {

				// Current node is not leaf
				if (!currentNode->isLeaf()) {
					BVHNode* left_node = nodes[currentNode->getIndex()];
					BVHNode* right_node = nodes[currentNode->getIndex() + 1];

					hit1 = left_node->getAABB().intercepts(ray, t1);
					hit2 = right_node->getAABB().intercepts(ray, t2);

					// Both nodes hit
					if (hit1 && hit2) {
						if (t1 < t2) {
							currentNode = left_node;
							hit_stack.push(StackItem(right_node, t2));
						}
						else {
							currentNode = right_node;
							hit_stack.push(StackItem(left_node, t1));
						}	
						continue;
					}

					// Only one node hits
					else if (hit1 || hit2) {
						currentNode = hit1 ? left_node : right_node;
						continue;
					}
				}

				// Current node is leaf
				else { 
					int n_obj = currentNode->getNObjs();
					int left_index = currentNode->getIndex();

					// Check intersection with every object in the current node
					for (int i = left_index; i < left_index + n_obj; i++) {
						hit1 = objects[i]->intercepts(ray, t1);
						if (hit1 && tmin > t1) {
							tmin = t1;
							closestObj = objects[i];
						}
					}
				}

				// Stack popping
				bool popped_item = false;
				while (!hit_stack.empty()) {
					StackItem item = hit_stack.top();
					hit_stack.pop();
					if (item.t < tmin || item.ptr->getAABB().isInside(ray.origin)) {
						currentNode = item.ptr;
						popped_item = true;
						break;
					}
				}

				// If we didn't pop a valid node and the stack is empty
				if (hit_stack.empty() && popped_item == false) {
					if (tmin == FLT_MAX) return(false);
					else {
						*hit_obj = closestObj;
						hit_point = ray.origin + (ray.direction * tmin);
						return(true);
					}
				}
			}

			return(false);
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			float tmp;
			bool hit1 = false, hit2;

			BVHNode* currentNode = nodes[0];

			// Intersect with the world box
			hit1 = currentNode->getAABB().intercepts(ray, tmp);
			if (!hit1) return false;

			while (true) {

				// Current node is not leaf
				if (!currentNode->isLeaf()) {
					BVHNode* left_node = nodes[currentNode->getIndex()];
					BVHNode* right_node = nodes[currentNode->getIndex() + 1];

					hit1 = left_node->getAABB().intercepts(ray, tmp);
					hit2 = right_node->getAABB().intercepts(ray, tmp);

					// Both nodes hit
					if (hit1 && hit2) {
						currentNode = left_node;
						hit_stack.push(StackItem(right_node, tmp));
						continue;
					}

					// Only one node hits
					else if (hit1 || hit2) {
						currentNode = hit1 ? left_node : right_node;
						continue;
					}
				}

				// Current node is leaf
				else {
					int n_obj = currentNode->getNObjs();
					int left_index = currentNode->getIndex();

					// Check intersection with every object in the current node
					for (int i = left_index; i < left_index + n_obj; i++) {
						hit1 = objects[i]->intercepts(ray, tmp);
						if (hit1 && tmp < length) return true;
					}
				}

				// If the stack is empty -> No intersection
				if (hit_stack.empty()) {
					return false;
				}

				// Stack popping
				StackItem item = hit_stack.top();
				currentNode = item.ptr;
				hit_stack.pop();
					
				
			}


			return(false);
	}		
