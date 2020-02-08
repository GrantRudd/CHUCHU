
(cl:in-package :asdf)

(defsystem "chuchu_workstation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Int16ArrayHeader" :depends-on ("_package_Int16ArrayHeader"))
    (:file "_package_Int16ArrayHeader" :depends-on ("_package"))
  ))