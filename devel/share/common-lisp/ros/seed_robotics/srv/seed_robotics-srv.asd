
(cl:in-package :asdf)

(defsystem "seed_robotics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BulkGetItem" :depends-on ("_package_BulkGetItem"))
    (:file "_package_BulkGetItem" :depends-on ("_package"))
    (:file "GetCurrent" :depends-on ("_package_GetCurrent"))
    (:file "_package_GetCurrent" :depends-on ("_package"))
    (:file "GetGoalPosition" :depends-on ("_package_GetGoalPosition"))
    (:file "_package_GetGoalPosition" :depends-on ("_package"))
    (:file "GetGoalVelocity" :depends-on ("_package_GetGoalVelocity"))
    (:file "_package_GetGoalVelocity" :depends-on ("_package"))
    (:file "GetPosVelocity" :depends-on ("_package_GetPosVelocity"))
    (:file "_package_GetPosVelocity" :depends-on ("_package"))
    (:file "GetPresentPosition" :depends-on ("_package_GetPresentPosition"))
    (:file "_package_GetPresentPosition" :depends-on ("_package"))
    (:file "GetPresentVelocity" :depends-on ("_package_GetPresentVelocity"))
    (:file "_package_GetPresentVelocity" :depends-on ("_package"))
    (:file "GetTempCurr" :depends-on ("_package_GetTempCurr"))
    (:file "_package_GetTempCurr" :depends-on ("_package"))
    (:file "GetTemperature" :depends-on ("_package_GetTemperature"))
    (:file "_package_GetTemperature" :depends-on ("_package"))
    (:file "SyncGetPosition" :depends-on ("_package_SyncGetPosition"))
    (:file "_package_SyncGetPosition" :depends-on ("_package"))
  ))