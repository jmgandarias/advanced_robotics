Eigen::Matrix4d ParsePoseMatrix(const YAML::Node &root, const std::string &key)
{
    // This function parses a 4x4 matrix from a YAML node given a specific key word.
    if (!root[key] || !root[key].IsSequence() || root[key].size() != 4)
    {
        throw std::runtime_error("YAML key '" + key + "' must be a 4x4 matrix sequence");
    }

    Eigen::Matrix4d pose;
    for (int row = 0; row < 4; ++row)
    {
        const YAML::Node row_node = root[key][row];
        if (!row_node.IsSequence() || row_node.size() != 4)
        {
            throw std::runtime_error("YAML key '" + key + "' must contain rows of length 4");
        }
        for (int col = 0; col < 4; ++col)
        {
            pose(row, col) = row_node[col].as<double>();
        }
    }

    return pose;
}