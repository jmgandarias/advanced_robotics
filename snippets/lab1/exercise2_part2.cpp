if (exercise_2)
{
    trajectory_msg.points.clear();
    const double sample_time = 0.1;
    int point_index = 0;

    const std::string package_name = "cartesian_trajectory_planning";
    const std::string package_share_dir =
        ament_index_cpp::get_package_share_directory(package_name);

    std::string output_base_dir = package_share_dir;
    const std::string install_suffix =
        "/install/" + package_name + "/share/" + package_name;
    const std::size_t suffix_pos = package_share_dir.rfind(install_suffix);
    if (suffix_pos != std::string::npos)
    {
        const std::string workspace_root = package_share_dir.substr(0, suffix_pos);
        const std::string source_candidate = workspace_root + "/src/" + package_name;

        struct stat source_candidate_stat;
        if (::stat(source_candidate.c_str(), &source_candidate_stat) == 0 &&
            S_ISDIR(source_candidate_stat.st_mode))
        {
            output_base_dir = source_candidate;
        }
    }

    const std::string output_dir = output_base_dir + "/experiment_data";

    if (::mkdir(output_dir.c_str(), 0755) != 0 && errno != EEXIST)
    {
        std::fprintf(stderr, "Failed to create output directory '%s': %s.\n", output_dir.c_str(), std::strerror(errno));
        return 1;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm;
    localtime_r(&now_time_t, &local_tm);

    std::ostringstream filename_builder;
    filename_builder << "data_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";
    const std::string output_csv = output_dir + "/" + filename_builder.str();
    std::ofstream csv_file(output_csv, std::ios::out | std::ios::trunc);
    if (!csv_file.is_open())
    {
        std::fprintf(stderr, "Failed to open CSV output file '%s'.\n", output_csv.c_str());
        return 1;
    }

    csv_file << "t,X,Y,Z,roll,pitch,yaw\n";
    csv_file << std::fixed << std::setprecision(9);