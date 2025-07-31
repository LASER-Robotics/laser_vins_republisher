#include <laser_vins_republisher/vins_republisher_node.hpp>

namespace laser_vins_republisher
{
    /**
     * @brief Construtor da classe VinsRepublisher.
     * Configura o nó, inicializa parâmetros e registra mensagens no log.
     * @param options Opções do nó ROS.
     */
    /* VinsRepublisher() //{ */
    VinsRepublisher::VinsRepublisher(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("vins_republisher", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating");

        declare_parameter("uav_name", rclcpp::ParameterValue(std::string("uav1")));

        declare_parameter("velocity_in_body_frame", rclcpp::ParameterValue(false));
        declare_parameter("init_in_zero", rclcpp::ParameterValue(true));
        declare_parameter("compensate_initial_tilt", rclcpp::ParameterValue(false));
        declare_parameter("rate_limiter.enabled", rclcpp::ParameterValue(false));
        declare_parameter("rate_limiter.max_rate", rclcpp::ParameterValue(10.0));

        declare_parameter("fcu_frame", rclcpp::ParameterValue(std::string("uav1/fcu")));
        declare_parameter("vins_frame", rclcpp::ParameterValue(std::string("uav1/ov_imu")));
        declare_parameter("vins_world_frame", rclcpp::ParameterValue(std::string("uav1/vins_world")));

        declare_parameter("static_transform.translation.x", rclcpp::ParameterValue(0.0));
        declare_parameter("static_transform.translation.y", rclcpp::ParameterValue(0.0));
        declare_parameter("static_transform.translation.z", rclcpp::ParameterValue(0.0));

        declare_parameter("static_transform.rotation.roll", rclcpp::ParameterValue(0.0));
        declare_parameter("static_transform.rotation.pitch", rclcpp::ParameterValue(0.0));
        declare_parameter("static_transform.rotation.yaw", rclcpp::ParameterValue(0.0));

        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);              ///< Broadcaster para enviar transformações.
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); ///< Broadcaster para enviar transformações estáticas.

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());              ///< Buffer para armazenar as transformações.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); ///< Listener para receber transformações.

        RCLCPP_INFO(get_logger(), "VinsRepublisher node initialized.");
    }
    /*//}*/

    /**
     * @brief Destrutor da classe VinsRepublisher.
     */
    /* ~VinsRepublisher() //{ */
    VinsRepublisher::~VinsRepublisher() {}
    /*//}*/

    /**
     * @brief Configura o nó no estado "configure".
     * Inicializa parâmetros, publishers, subscribers, timers e serviços.
     * @return CallbackReturn::SUCCESS se configurado com sucesso.
     */
    /* on_configure() //{ */
    CallbackReturn VinsRepublisher::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring VinsRepublisher...");

        getParameters();          // Carrega os parâmetros do nó.
        configPubSub();           // Configura os publishers e subscribers.
        configTimers();           // Configura os timers.
        configServices();         // Configura os serviços.
        publishStaticTransform(); // Publica a transformação estática entre os quadros de referência.

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Ativa o nó no estado "activate".
     * Habilita o publisher e marca o nó como ativo.
     * @return CallbackReturn::SUCCESS se ativado com sucesso.
     */
    /* on_activate() //{ */
    CallbackReturn VinsRepublisher::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating VinsRepublisher..."); // Loga mensagem indicando a ativação do nó.

        pub_odometry->on_activate();

        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = true;
        }

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Desativa o nó no estado "deactivate".
     * Desabilita o publisher e marca o nó como inativo.
     * @return CallbackReturn::SUCCESS se desativado com sucesso.
     */
    /* on_deactivate() //{ */
    CallbackReturn VinsRepublisher::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating VinsRepublisher...");
        pub_odometry->on_deactivate(); // Desativa o publisher.
        {
            std::lock_guard<std::mutex> lock(mtx_);
            is_active_ = false;
        }

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Realiza limpeza de recursos no estado "cleanup".
     * Libera memória e recursos associados aos publishers e subscribers.
     * @return CallbackReturn::SUCCESS se limpo com sucesso.
     */
    /* on_cleanup() //{ */
    CallbackReturn VinsRepublisher::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up VinsRepublisher...");

        sub_odometry.reset(); // Limpa o subscriber.
        pub_odometry.reset(); // Limpa o publisher.

        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Realiza ações de desligamento no estado "shutdown".
     * @return CallbackReturn::SUCCESS se desligado com sucesso.
     */
    /* on_shutdown() //{ */
    CallbackReturn VinsRepublisher::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down VinsRepublisher...");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /**
     * @brief Carrega parâmetros configurados do arquivo de configuração do ROS 2.
     */
    /* getParameters() //{ */
    void VinsRepublisher::getParameters()
    {
        get_parameter("uav_name", _uav_name_);

        get_parameter("velocity_in_body_frame", _velocity_in_body_frame_);
        get_parameter("init_in_zero", _init_in_zero_);
        get_parameter("compensate_initial_tilt", _compensate_initial_tilt_);
        get_parameter("rate_limiter.enabled", _rate_limiter_);
        get_parameter("rate_limiter.max_rate", _rate_limiter_max_);

        get_parameter("fcu_frame", _fcu_frame_);
        get_parameter("vins_frame", _vins_fcu_frame_);
        get_parameter("vins_world_frame", _vins_world_frame_);

        _fcu_frame_ = _uav_name_ + "/" + _fcu_frame_;
        _vins_fcu_frame_ = _uav_name_ + "/" + _vins_fcu_frame_;
        _vins_world_frame_ = _uav_name_ + "/" + _vins_world_frame_;

        double x, y, z, roll, pitch, yaw;
        get_parameter("static_transform.translation.x", x);
        get_parameter("static_transform.translation.y", y);
        get_parameter("static_transform.translation.z", z);
        _translation_ = Eigen::Vector3d(x, y, z);

        get_parameter("static_transform.rotation.roll", roll);
        get_parameter("static_transform.rotation.pitch", pitch);
        get_parameter("static_transform.rotation.yaw", yaw);
        _rotation_ = Eigen::Vector3d(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(), "Parameters loaded.");
    }
    /*//}*/

    /**
     * @brief Configura os publishers e subscribers para comunicação.
     */
    /* configPubSub() //{ */
    void VinsRepublisher::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Initializing publishers and subscribers...");

        // Configura o subscriber para receber mensagens do tópico.
        sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
            "odometry_in", 10, std::bind(&VinsRepublisher::odometryCallback, this, std::placeholders::_1));

        publisher_odom_last_published_ = get_clock()->now();

        // Configura o publisher para enviar mensagens ao tópico.
        pub_odometry = create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);
    }
    /*//}*/

    /**
     * @brief Configura os timers para execução periódica da lógica do nó.
     */
    /* configTimers() //{ */
    void VinsRepublisher::configTimers()
    {
        RCLCPP_INFO(get_logger(), "Initializing timers...");
    }
    /*//}*/

    /**
     * @brief Configura os serviços oferecidos pelo nó.
     */
    /* configServices() //{ */
    void VinsRepublisher::configServices()
    {
        RCLCPP_INFO(get_logger(), "Initializing services..."); // Loga mensagem indicando a inicialização dos serviços.
    }
    /*//}*/

    /**
     * @brief Publica a transformação estática entre os quadros de referência.
     * @param _fcu_frame_ Quadro de referência da aeronave.
     * @param _vins_fcu_frame_ Quadro de referência do IMU.
     * @param _translation_ Vetor de translação.
     * @param _rotation_ Vetor de Rotação.
     */
    /* publishStaticTransform() //{ */
    void VinsRepublisher::publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = get_clock()->now();
        static_transform.header.frame_id = _fcu_frame_;
        static_transform.child_frame_id = _vins_fcu_frame_;

        static_transform.transform.translation.x = _translation_.x();
        static_transform.transform.translation.y = _translation_.y();
        static_transform.transform.translation.z = _translation_.z();

        tf2::Quaternion q;
        q.setRPY(_rotation_.x(), _rotation_.y(), _rotation_.z());
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(static_transform);

        RCLCPP_INFO(get_logger(), "Published static transform from '%s' to '%s' with translation (%.3f, %.3f, %.3f) and rotation (%.3f, %.3f, %.3f)",
                    static_transform.header.frame_id.c_str(), static_transform.child_frame_id.c_str(), _translation_.x(), _translation_.y(), _translation_.z(), _rotation_.x(), _rotation_.y(), _rotation_.z());
    }
    /*//}*/

    /**
     * @brief Callback chamado quando uma mensagem de odometria é recebida.
     * @param msg Mensagem de odometria recebida.
     */
    void VinsRepublisher::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!validateOdometry(*msg))
        {
            RCLCPP_ERROR(get_logger(), "input odometry is not numerically valid");
            return;
        }

        if (_rate_limiter_ &&
            (get_clock()->now() - publisher_odom_last_published_).seconds() < (1.0 / _rate_limiter_max_))
        {
            RCLCPP_DEBUG(get_logger(), "skipping over");
            return;
        }

        if (!tf_buffer_->canTransform(_vins_fcu_frame_, _fcu_frame_, msg->header.stamp))
        {
            RCLCPP_WARN(get_logger(), "could not find transform from '%s' to '%s'", _fcu_frame_.c_str(), _vins_fcu_frame_.c_str());
            return;
        }

        geometry_msgs::msg::TransformStamped T_IMU_FCU = tf_buffer_->lookupTransform(_vins_fcu_frame_, _fcu_frame_, msg->header.stamp);

        nav_msgs::msg::Odometry odom_transformed;
        odom_transformed.header = msg->header;
        odom_transformed.header.frame_id = _vins_world_frame_;
        odom_transformed.child_frame_id = _fcu_frame_;

        geometry_msgs::msg::Pose pose_transformed;
        // tf2::Quaternion R_GLOBAL_IMU, R_IMU_FCU;

        Eigen::Matrix3d R_IMU_FCU = laser_uav_lib::AttitudeConverter(T_IMU_FCU.transform.rotation);
        Eigen::Matrix3d R_GLOBAL_IMU = laser_uav_lib::AttitudeConverter(msg->pose.pose.orientation);
        Eigen::Matrix3d R_GLOBAL_FCU = laser_uav_lib::AttitudeConverter(R_GLOBAL_IMU * R_IMU_FCU);
        pose_transformed.orientation = laser_uav_lib::AttitudeConverter(R_GLOBAL_IMU * R_IMU_FCU);

        Eigen::Vector3d t_IMU_FCU(T_IMU_FCU.transform.translation.x, T_IMU_FCU.transform.translation.y, T_IMU_FCU.transform.translation.z);

        Eigen::Vector3d translation = R_GLOBAL_IMU * t_IMU_FCU;

        pose_transformed.position.x = msg->pose.pose.position.x + translation(0);
        pose_transformed.position.y = msg->pose.pose.position.y + translation(1);
        pose_transformed.position.z = msg->pose.pose.position.z + translation(2);

        geometry_msgs::msg::TransformStamped tf_msg;
        if (_init_in_zero_)
        {
            if (!got_init_pose_)
            {
                init_hdg_ = laser_uav_lib::AttitudeConverter(pose_transformed.orientation).getHeading();
                RCLCPP_INFO(get_logger(), "Initial heading: %.2f", init_hdg_);
                got_init_pose_ = true;
            }

            tf_msg.header.stamp = msg->header.stamp;
            tf_msg.header.frame_id = odom_transformed.header.frame_id;
            tf_msg.child_frame_id = msg->header.frame_id;

            tf_msg.transform.translation.x = 0.0;
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = laser_uav_lib::AttitudeConverter(0, 0, 0).setHeading(-init_hdg_);

            tf2::doTransform(pose_transformed, pose_transformed, tf_msg);

            tf2::Transform tf;
            tf2::fromMsg(tf_msg.transform, tf);
            odom_transformed.pose.covariance = transformCovariance(msg->pose.covariance, tf);
        }
        else

        {
            odom_transformed.pose.covariance = msg->pose.covariance;
        }

        geometry_msgs::msg::TransformStamped tf_msg_inv;
        tf_msg_inv.header.stamp = msg->header.stamp;
        tf_msg_inv.header.frame_id = msg->header.frame_id;
        tf_msg_inv.child_frame_id = odom_transformed.header.frame_id;
        tf_msg_inv.transform.translation.x = 0;
        tf_msg_inv.transform.translation.y = 0;
        tf_msg_inv.transform.translation.z = 0;
        tf_msg_inv.transform.rotation = laser_uav_lib::AttitudeConverter(0, 0, 0).setHeading(init_hdg_);

        try
        {
            broadcaster_->sendTransform(tf_msg_inv);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(), "Could not send transform: %s", ex.what());
        }

        odom_transformed.pose.pose = pose_transformed;

        tf2::Transform tf;
        tf.setOrigin(tf2::Vector3(0, 0, 0));
        tf.setRotation(laser_uav_lib::AttitudeConverter(0, 0, 0).setHeading(-init_hdg_));

        geometry_msgs::msg::Vector3 linear_velocity = msg->twist.twist.linear;
        geometry_msgs::msg::Vector3 angular_velocity = msg->twist.twist.angular;

        if (_velocity_in_body_frame_)
        {
            Eigen::Vector3d v2;
            v2 << linear_velocity.x, linear_velocity.y, linear_velocity.z;

            v2 = R_IMU_FCU.transpose() * v2;

            /* v2 = R_GLOBAL_FCU.transpose() * v2; */
            linear_velocity.x = v2(0);
            linear_velocity.y = v2(1);
            linear_velocity.z = v2(2);

            Eigen::Vector3d v3;
            v3 << angular_velocity.x, angular_velocity.y, angular_velocity.z;
            v3 = R_IMU_FCU.transpose() * v3;
            /* v3 = R_GLOBAL_FCU.transpose() * v3; */
            angular_velocity.x = v3(0);
            angular_velocity.y = v3(1);
            angular_velocity.z = v3(2);
        }
        else
        {
            Eigen::Vector3d v2;
            v2 << linear_velocity.x, linear_velocity.y, linear_velocity.z;
            v2 = R_GLOBAL_FCU.transpose() * v2;
            linear_velocity.x = v2(0);
            linear_velocity.y = v2(1);
            linear_velocity.z = v2(2);

            Eigen::Vector3d v3;
            v3 << angular_velocity.x, angular_velocity.y, angular_velocity.z;
            v3 = R_GLOBAL_FCU.transpose() * v3;
            angular_velocity.x = v3(0);
            angular_velocity.y = v3(1);
            angular_velocity.z = v3(2);
        }

        odom_transformed.twist.twist.linear = linear_velocity;
        odom_transformed.twist.twist.angular = angular_velocity;
        odom_transformed.twist.covariance = transformCovariance(msg->twist.covariance, tf);

        if (!validateOdometry(odom_transformed))
        {
            RCLCPP_ERROR(get_logger(), "output odometry is not numerically valid");
            return;
        }

        if (_compensate_initial_tilt_)
        {
            if (is_calibrated_)
            {
                odom_transformed.pose.pose.position.x -= odom_init_.pose.pose.position.x;
                odom_transformed.pose.pose.position.y -= odom_init_.pose.pose.position.y;
                odom_transformed.pose.pose.position.z -= odom_init_.pose.pose.position.z;

                Eigen::Vector3d pos;
                Eigen::Matrix3d init_rot = laser_uav_lib::AttitudeConverter(odom_init_.pose.pose.orientation);
                pos << odom_transformed.pose.pose.position.x, odom_transformed.pose.pose.position.y, odom_transformed.pose.pose.position.z;
                pos = init_rot.inverse() * pos;

                const Eigen::Quaterniond q_init_rot = laser_uav_lib::AttitudeConverter(init_rot);
                const Eigen::Quaterniond q_curr_rot = laser_uav_lib::AttitudeConverter(odom_transformed.pose.pose.orientation);
                const Eigen::Quaterniond q_calibrated = q_init_rot * q_curr_rot;

                odom_transformed.pose.pose.orientation = laser_uav_lib::AttitudeConverter(q_calibrated);
            }
            else
            {
                {
                    std::lock_guard<std::mutex> lock(mtx_odom_init_);
                    odom_init_ = odom_transformed;
                }
                auto [roll, pitch, yaw] = laser_uav_lib::AttitudeConverter(odom_init_.pose.pose.orientation).getExtrinsicRPY();

                RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 1000, "init_odom: t: (%.2f, %.2f, %.2f) [m] rpy: (%.2f, %.2f, %.2f) [deg], waiting for calibration service call", odom_init_.pose.pose.position.x, odom_init_.pose.pose.position.y, odom_init_.pose.pose.position.z, roll * 180 / 3.14,
                                     pitch * 180 / 3.14, yaw * 180 / 3.14);

                has_valid_odom_ = true;
                return;
            }
        }

        try
        {
            pub_odometry->publish(odom_transformed);
            // RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 10000, "Publishing");
            publisher_odom_last_published_ = get_clock()->now();
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Error publishing odometry message");
        }
    }

    /**
     * @brief Define o heading de uma orientação.
     * @param q_msg Orientação a ser modificada.
     * @param heading Ângulo de heading desejado.
     * @throws std::runtime_error se uma singularidade for detectada.
     * @return Orientação com o heading definido.
     * @note A singularidade ocorre quando o vetor Z da orientação está muito próximo do plano XY.
     */
    /* setHeading() //{ */
    geometry_msgs::msg::Quaternion VinsRepublisher::setHeading(tf2::Quaternion &q_msg, double heading)
    {

        Eigen::Quaterniond q(q_msg.w(), q_msg.x(), q_msg.y(), q_msg.z());
        Eigen::Matrix3d R = q.toRotationMatrix();

        // Obtém o vetor Z da rotação original
        Eigen::Vector3d b3 = R.col(2);

        // Verifica singularidade: se o vetor Z estiver muito próximo do plano XY, não podemos definir heading
        if (fabs(b3[2]) < 1e-3)
        {
            throw std::runtime_error("Singularidade detectada ao definir o heading.");
        }

        // Cria um vetor que representa o heading desejado no plano XY
        Eigen::Vector3d h(cos(heading), sin(heading), 0);

        Eigen::Matrix3d new_R;
        new_R.col(2) = b3;

        // Projeção oblíqua
        Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - b3 * b3.transpose();
        Eigen::MatrixXd A = projector.leftCols(2);
        Eigen::MatrixXd B(3, 2);
        B << 1, 0, 0, 1, 0, 0;

        Eigen::MatrixXd Bt_A = B.transpose() * A;
        Eigen::MatrixXd Bt_A_pseudoinverse = (Bt_A.transpose() * Bt_A).inverse() * Bt_A.transpose();
        Eigen::MatrixXd oblique_projector = A * Bt_A_pseudoinverse * B.transpose();

        new_R.col(0) = oblique_projector * h;
        new_R.col(0).normalize();

        // Calcula o vetor Y ortogonal aos outros dois
        new_R.col(1) = new_R.col(2).cross(new_R.col(0));
        new_R.col(1).normalize();

        // Converte de volta para quaternion
        Eigen::Quaterniond new_q(new_R);
        geometry_msgs::msg::Quaternion msg_quaternion;
        msg_quaternion.x = new_q.x();
        msg_quaternion.y = new_q.y();
        msg_quaternion.z = new_q.z();
        msg_quaternion.w = new_q.w();

        return msg_quaternion;
    }
    /*//}*/

    /**
     * @brief Valida a mensagem de odometria recebida.
     * @param odometry Mensagem de odometria recebida.
     * @return true se a mensagem é válida, false caso contrário.
     */
    /* validateOdometry() //{ */
    bool VinsRepublisher::validateOdometry(const nav_msgs::msg::Odometry &odometry)
    {

        // check position

        if (!std::isfinite(odometry.pose.pose.position.x))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.position.x'!!!");
            return false;
        }

        if (!std::isfinite(odometry.pose.pose.position.y))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.position.y'!!!");
            return false;
        }

        if (!std::isfinite(odometry.pose.pose.position.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.position.z'!!!");
            return false;
        }

        // check orientation

        if (!std::isfinite(odometry.pose.pose.orientation.x))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.orientation.x'!!!");
            return false;
        }

        if (!std::isfinite(odometry.pose.pose.orientation.y))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.orientation.y'!!!");
            return false;
        }

        if (!std::isfinite(odometry.pose.pose.orientation.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.orientation.z'!!!");
            return false;
        }

        if (!std::isfinite(odometry.pose.pose.orientation.w))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.pose.pose.orientation.w'!!!");
            return false;
        }
        // check if the quaternion is sound

        if (fabs(Eigen::Vector4d(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
                                 odometry.pose.pose.orientation.w)
                     .norm() -
                 1.0) > 1e-2)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "orientation is not sound!!!");
            return false;
        }

        // check velocity

        if (!std::isfinite(odometry.twist.twist.linear.x))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.twist.twist.linear.x'!!!");
            return false;
        }

        if (!std::isfinite(odometry.twist.twist.linear.y))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.twist.twist.linear.y'!!!");
            return false;
        }

        if (!std::isfinite(odometry.twist.twist.linear.z))
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 1000, "NaN detected in variable 'odometry.twist.twist.linear.z'!!!");
            return false;
        }

        return true;
    }
    /*//}*/

    /**
     * @brief Valida a orientação recebida.
     * @param tf2_quaternion_ Orientação recebida.
     * @return true se a orientação é válida, false caso contrário.
     */
    /* validateOrientation() //{ */
    bool VinsRepublisher::validateOrientation(tf2::Quaternion tf2_quaternion_)
    {
        if (!std::isfinite(tf2_quaternion_.x()) || !std::isfinite(tf2_quaternion_.y()) || !std::isfinite(tf2_quaternion_.z()) ||
            !std::isfinite(tf2_quaternion_.w()))
        {
            return false;
        }
        return true;
    }
    /*//}*/

    /**
     * @brief Transforma uma matriz de covariância de uma pose.
     * @param cov_in Matriz de covariância de entrada.
     * @param transform Transformação a ser aplicada.
     * @return Matriz de covariância transformada.
     * @note A matriz de covariância é transformada de acordo com a transformação aplicada à pose.
     */
    /* transformCovariance() //{ */
    geometry_msgs::msg::PoseWithCovariance::_covariance_type VinsRepublisher::transformCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type &cov_in,
                                                                                                  const tf2::Transform &transform)
    {
        /**
         * To transform a covariance matrix:
         *
         * [R 0] COVARIANCE [R' 0 ]
         * [0 R]            [0  R']
         *
         * Where:
         * 	R is the rotation matrix (3x3).
         * 	R' is the transpose of the rotation matrix.
         * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
         */

        // get rotation matrix transpose
        const tf2::Matrix3x3 R_transpose = transform.getBasis().transpose();

        // convert the covariance matrix into four 3x3 blocks
        const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2], cov_in[6], cov_in[7], cov_in[8], cov_in[12], cov_in[13], cov_in[14]);
        const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5], cov_in[9], cov_in[10], cov_in[11], cov_in[15], cov_in[16], cov_in[17]);
        const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20], cov_in[24], cov_in[25], cov_in[26], cov_in[30], cov_in[31], cov_in[32]);
        const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23], cov_in[27], cov_in[28], cov_in[29], cov_in[33], cov_in[34], cov_in[35]);

        // perform blockwise matrix multiplication
        const tf2::Matrix3x3 result_11 = transform.getBasis() * cov_11 * R_transpose;
        const tf2::Matrix3x3 result_12 = transform.getBasis() * cov_12 * R_transpose;
        const tf2::Matrix3x3 result_21 = transform.getBasis() * cov_21 * R_transpose;
        const tf2::Matrix3x3 result_22 = transform.getBasis() * cov_22 * R_transpose;

        // form the output
        geometry_msgs::msg::PoseWithCovariance::_covariance_type output;
        output[0] = result_11[0][0];
        output[1] = result_11[0][1];
        output[2] = result_11[0][2];
        output[6] = result_11[1][0];
        output[7] = result_11[1][1];
        output[8] = result_11[1][2];
        output[12] = result_11[2][0];
        output[13] = result_11[2][1];
        output[14] = result_11[2][2];

        output[3] = result_12[0][0];
        output[4] = result_12[0][1];
        output[5] = result_12[0][2];
        output[9] = result_12[1][0];
        output[10] = result_12[1][1];
        output[11] = result_12[1][2];
        output[15] = result_12[2][0];
        output[16] = result_12[2][1];
        output[17] = result_12[2][2];

        output[18] = result_21[0][0];
        output[19] = result_21[0][1];
        output[20] = result_21[0][2];
        output[24] = result_21[1][0];
        output[25] = result_21[1][1];
        output[26] = result_21[1][2];
        output[30] = result_21[2][0];
        output[31] = result_21[2][1];
        output[32] = result_21[2][2];

        output[21] = result_22[0][0];
        output[22] = result_22[0][1];
        output[23] = result_22[0][2];
        output[27] = result_22[1][0];
        output[28] = result_22[1][1];
        output[29] = result_22[1][2];
        output[33] = result_22[2][0];
        output[34] = result_22[2][1];
        output[35] = result_22[2][2];

        return output;
    }
    /*//}*/

} // namespace uav_waypoint
