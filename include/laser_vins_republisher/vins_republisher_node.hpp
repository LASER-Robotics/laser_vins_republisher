#ifndef LASER_VINS_REPUBLISHER__VINS_REPUBLISHER_HPP_
#define LASER_VINS_REPUBLISHER__VINS_REPUBLISHER_HPP_

/* includes //{ */
#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

/*//}*/

/* define //{*/
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/*//}*/

namespace laser_vins_republisher
{
    /**
     * @class VinsRepublisher
     * @brief Classe principal para carregar e gerenciar waypoints de um UAV.
     */
    class VinsRepublisher : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief Construtor da classe VinsRepublisher.
         * @param options Opções do nó ROS 2.
         */
        /* VinsRepublisher() //{ */
        explicit VinsRepublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        /*//}*/

        /**
         * @brief Destrutor da classe VinsRepublisher.
         */
        /* ~VinsRepublisher() //{ */
        ~VinsRepublisher() override;
        /*//}*/

    private:
        /* CONFIG //{ */

        /**
         * @brief Configura o nó no estado "configure".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da configuração.
         */
        /* on_configure() //{ */
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Ativa o nó no estado "activate".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da ativação.
         */
        /* on_activate() //{ */
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Desativa o nó no estado "deactivate".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da desativação.
         */
        /* on_deactivate() //{ */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Limpa os recursos no estado "cleanup".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado da limpeza.
         */
        /* on_cleanup() //{ */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Realiza ações no estado "shutdown".
         * @param state Estado atual do ciclo de vida.
         * @return Resultado do desligamento.
         */
        /* on_shutdown() //{ */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        /*//}*/

        /**
         * @brief Obtém parâmetros configurados no arquivo de configuração.
         */
        /* getParameters() //{ */
        void getParameters();
        /*//}*/

        /**
         * @brief Configura publishers e subscribers para comunicação ROS.
         */
        /* configPubSub() //{ */
        void configPubSub();
        /*//}*/

        /**
         * @brief Configura os timers para execução periódica de tarefas.
         */
        /* configTimers() //{ */
        void configTimers();
        /*//}*/

        /**
         * @brief Configura os serviços oferecidos pelo nó.
         */
        /* configServices() //{ */
        void configServices();
        /*//}*/
        /*//}*/

        /* SUBSCRIBERS //{ */

        /* odometryCallback() //{ */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry; ///< Subscriber para receber mensagens de odometria.

        /**
         * @brief Callback chamado quando uma mensagem indicando a existência de um objetivo é recebida.
         * @param msg Mensagem recebida.
         */
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        rclcpp::Time publisher_odom_last_published_; ///< Último horário de publicação da mensagem de odometria.

        /*//}*/

        /*//}*/

        /* PUBLISHERS //{ */
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry; ///< Publisher para enviar mensagens de odometria.

        /*//}*/

        /* VARIAVEIS GLOBAIS //{ */

        std::mutex mtx_;        //< Mutex para sincronização de acesso.
        bool is_active_{false}; //< Indica se o nó está ativo.

        bool _velocity_in_body_frame_{false};  ///< Define se as velocidades estão no referencial do corpo (true) ou global (false).
        bool _init_in_zero_{false};            ///< Indica se o primeiro ângulo de orientação deve ser subtraído para definir a origem em zero.
        bool got_init_pose_{false};            ///< Indica se a posição inicial foi recebida.
        bool _compensate_initial_tilt_{false}; ///< Indica se deve compensar a inclinação inicial para alinhar com a gravidade.
        bool _rate_limiter_{false};            ///< Habilita ou desabilita o limitador de taxa de processamento.
        bool has_valid_odom_{false};           ///< Habilita ou desabilita o limitador de taxa de processamento.
        bool is_calibrated_{false};            ///< Habilita ou desabilita o limitador de taxa de processamento.
        double _rate_limiter_max_;             ///< Define a taxa máxima de processamento (em Hz) quando o limitador está ativado.

        std::string _uav_name_;  ///< Nome da aeronave.
        std::string _uav_frame_; ///< Quadro de referência da aeronave.

        std::string _fcu_frame_;        ///< Quadro de referência do FCU.
        std::string _vins_fcu_frame_;   ///< Quadro de referência do VINS.
        std::string _vins_world_frame_; ///< Quadro de referência do World.

        Eigen::Vector3d _translation_; ///< Translação entre os quadros de referência.
        Eigen::Vector3d _rotation_;    ///< Rotação entre os quadros de referência.
        double init_hdg_;              ///< Ângulo de orientação inicial.

        nav_msgs::msg::Odometry odom_init_; ///< Mensagem de odometria inicial.
        std::mutex mtx_odom_init_;          ///< Mutex para sincronização de acesso à mensagem de odometria inicial.

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                              ///< Buffer para armazenar as transformações.
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                 ///< Listener para receber transformações.
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; ///< Broadcaster para enviar transformações estáticas.
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;              ///< Broadcaster para enviar transformações.
        /*//}*/

        /* TIMERS //{ */

        /*//}*/

        /* SERVICES //{ */

        /*//}*/

        /*//}*/

        /* FUNCIONS //{ */
        /**
         * @brief Valida a mensagem de odometria recebida.
         * @param odometry Mensagem de odometria recebida.
         * @return true se a mensagem é válida, false caso contrário.
         */
        bool
        validateOdometry(const nav_msgs::msg::Odometry &odometry);

        /**
         * @brief Publica a transformação estática entre os quadros de referência.
         * @param _fcu_frame_ Quadro de referência da aeronave.
         * @param _vins_imu_frame_ Quadro de referência do IMU.
         * @param _translation_ Vetor de translação.
         * @param _rotation_ Vetor de Rotação.
         */
        void publishStaticTransform();

        /**
         * @brief Valida a orientação recebida.
         * @param tf2_quaternion_ Orientação recebida.
         * @return true se a orientação é válida, false caso contrário.
         */
        bool validateOrientation(tf2::Quaternion tf2_quaternion_);

        /**
         * @brief Define o heading de uma orientação.
         * @param q_msg Orientação a ser modificada.
         * @param heading Ângulo de heading desejado.
         * @throws std::runtime_error se uma singularidade for detectada.
         * @return Orientação com o heading definido.
         * @note A singularidade ocorre quando o vetor Z da orientação está muito próximo do plano XY.
         */
        geometry_msgs::msg::Quaternion setHeading(tf2::Quaternion &q_msg, double heading);

        /**
         * @brief Transforma uma matriz de covariância de uma pose.
         * @param cov_in Matriz de covariância de entrada.
         * @param transform Transformação a ser aplicada.
         * @return Matriz de covariância transformada.
         * @note A matriz de covariância é transformada de acordo com a transformação aplicada à pose.
         */
        geometry_msgs::msg::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type &cov_in,
                                                                                     const tf2::Transform &transform);

        /*//}*/
    };
} // namespace laser_vins_republisher

#endif // LASER_VINS_REPUBLISHER__VINS_REPUBLISHER_HPP_
