
(cl:in-package :asdf)

(defsystem "hybrid_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SceneComplexity" :depends-on ("_package_SceneComplexity"))
    (:file "_package_SceneComplexity" :depends-on ("_package"))
    (:file "TetherStatus" :depends-on ("_package_TetherStatus"))
    (:file "_package_TetherStatus" :depends-on ("_package"))
  ))